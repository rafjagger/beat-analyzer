/**
 * Beat Analyzer - Hauptanwendung
 * 
 * Eigenständige Anwendung für:
 * - Audio Input via JACK (separate BPM und VU Kanäle)
 * - Beat Detection (eigene Implementierung)
 * - OSC Output (Beat Clock + VU-Meter)
 * 
 * Keine Mixxx-Abhängigkeiten - komplett eigenständig!
 */

#include "audio/jack_client.h"
#include "analysis/beat_detection.h"
#include "analysis/vu_meter.h"
#include "osc/osc_sender.h"
#include "osc/osc_receiver.h"
#include "config/config_loader.h"
#include "config/env_config.h"
#include "util/logging.h"

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <mutex>
#include <vector>
#include <cmath>
#include <regex>

using namespace BeatAnalyzer;
using namespace BeatAnalyzer::Audio;
using namespace BeatAnalyzer::OSC;
using namespace BeatAnalyzer::Config;
using namespace BeatAnalyzer::Util;

// ============================================================================
// Globale Signal-Handling
// ============================================================================

std::atomic<bool> g_running(true);

void signalHandler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        LOG_INFO("Shutdown signal empfangen...");
        g_running = false;
    }
}

// ============================================================================
// Helper: Parse host:port string
// ============================================================================

bool parseHostPort(const std::string& value, std::string& host, int& port) {
    // Format: "host:port" or just "host" (default port 9000)
    auto colonPos = value.rfind(':');
    if (colonPos != std::string::npos) {
        host = value.substr(0, colonPos);
        try {
            port = std::stoi(value.substr(colonPos + 1));
            return true;
        } catch (...) {
            return false;
        }
    } else {
        host = value;
        port = 9000;
        return true;
    }
}

// ============================================================================
// Beat Analyzer Anwendung
// ============================================================================

class BeatAnalyzerApp {
public:
    BeatAnalyzerApp() : m_numBpmChannels(4), m_numVuChannels(2), m_frameCount(0) {}
    
    bool initialize(const std::string&) {
        LOG_INFO("Beat Analyzer wird initialisiert...");
        
        // .env Konfiguration laden
        auto& env = EnvConfig::instance();
        if (env.load(".env") || env.load("../.env")) {
            LOG_INFO(".env Konfiguration geladen");
        } else if (env.load(".env.example")) {
            LOG_INFO(".env.example als Fallback geladen");
        }
        
        // Log-Level setzen
        int logLevel = env.getInt("LOG_LEVEL", 1);
        Logger::setLogLevel(static_cast<LogLevel>(logLevel));
        
        // Anzahl Kanäle
        m_numBpmChannels = std::max(0, env.getInt("NUM_BPM_CHANNELS", 4));
        m_numVuChannels = std::max(0, env.getInt("NUM_VU_CHANNELS", 2));
        
        LOG_INFO("BPM Kanäle: " + std::to_string(m_numBpmChannels) + 
                 ", VU Kanäle: " + std::to_string(m_numVuChannels));
        
        // Feature Toggles
        m_enableBeatclock = env.getInt("ENABLE_BEATCLOCK", 1) != 0;
        m_enableVu = env.getInt("ENABLE_VU", 1) != 0;
        
        m_enableBeat = env.getInt("ENABLE_BEAT", 1) != 0;
        
        // VU-Meter Konfiguration
        m_vuRmsAttack = env.getFloat("VU_RMS_ATTACK", 0.8f);
        m_vuRmsRelease = env.getFloat("VU_RMS_RELEASE", 0.2f);
        m_vuPeakFalloff = env.getFloat("VU_PEAK_FALLOFF", 20.0f);
        
        // Beat Detection Latenz-Kompensation
        // FFT analysiert Mitte des Fensters (512 samples) + Peak-Detection (512 samples)
        // Bei 44100 Hz: (512 + 512) / 44100 = ~23ms
        m_beatLatencyMs = env.getFloat("BEAT_LATENCY_MS", 23.0f);
        m_beatLatencyFrames = static_cast<int64_t>(m_beatLatencyMs * 44.1f);  // samples
        
        LOG_INFO("Features: Beatclock=" + std::string(m_enableBeatclock ? "ON" : "OFF") +
                 " VU=" + std::string(m_enableVu ? "ON" : "OFF") +
                 " Beat=" + std::string(m_enableBeat ? "ON" : "OFF") +
                 " Latenz-Kompensation=" + std::to_string(static_cast<int>(m_beatLatencyMs)) + "ms");
        
        // Beat Detector Konfiguration
        BeatDetectorConfig beatConfig;
        beatConfig.sampleRate = 44100;
        beatConfig.frameSize = 1024;
        beatConfig.hopSize = 512;
        m_bpmMin = env.getFloat("BPM_MIN", 60.0f);
        m_bpmMax = env.getFloat("BPM_MAX", 200.0f);
        beatConfig.minBpm = m_bpmMin;
        beatConfig.maxBpm = m_bpmMax;
        beatConfig.defaultBpm = 120.0;
        
        // Beat Tracker für BPM Kanäle
        for (int i = 0; i < m_numBpmChannels; ++i) {
            auto tracker = std::make_unique<RealTimeBeatTracker>(beatConfig);
            if (!tracker->initialize()) {
                LOG_ERROR("Beat Tracker " + std::to_string(i+1) + " konnte nicht initialisiert werden");
                return false;
            }
            m_beatTrackers.push_back(std::move(tracker));
            m_bpmTrackStates.push_back(BpmTrackState{});
        }
        
        // VU-Meter für VU Kanäle
        for (int i = 0; i < m_numVuChannels; ++i) {
            auto vuMeter = std::make_unique<VuMeter>();
            vuMeter->setRmsAttack(m_vuRmsAttack);
            vuMeter->setRmsRelease(m_vuRmsRelease);
            vuMeter->setPeakFalloff(m_vuPeakFalloff);
            m_vuMeters.push_back(std::move(vuMeter));
            m_vuTrackStates.push_back(VuTrackState{});
        }
        
        LOG_INFO(std::to_string(m_numBpmChannels) + " Beat Tracker, " + 
                 std::to_string(m_numVuChannels) + " VU-Meter initialisiert");
        
        // JACK Client initialisieren
        std::string jackName = env.getString("JACK_CLIENT_NAME", "beat-analyzer");
        m_jackClient = std::make_shared<JackClient>(jackName, m_numBpmChannels, m_numVuChannels);
        
        if (!m_jackClient->initialize()) {
            LOG_ERROR("JACK Client konnte nicht initialisiert werden");
            return false;
        }
        
        // OSC Sender initialisieren - suche nach OSC_HOST_* Einträgen
        m_oscSender = std::make_shared<OscSender>();
        
        auto oscHostKeys = env.getKeysWithPrefix("OSC_HOST_");
        if (!oscHostKeys.empty()) {
            // Mehrere Hosts: OSC_HOST_1, OSC_HOST_2, etc.
            for (const auto& key : oscHostKeys) {
                std::string value = env.getString(key, "");
                if (value.empty()) continue;
                
                std::string host;
                int port;
                if (parseHostPort(value, host, port)) {
                    // Extrahiere Namen aus Key (OSC_HOST_Protokol -> Protokol)
                    std::string name = key.substr(9);  // Nach "OSC_HOST_"
                    m_oscSender->addTarget(name, host, port);
                }
            }
        } else {
            // Fallback: alte einzelne OSC_HOST/OSC_PORT Variablen
            std::string oscHost = env.getString("OSC_HOST", "127.0.0.1");
            int oscPort = env.getInt("OSC_PORT", 9000);
            m_oscSender->addTarget("default", oscHost, oscPort);
        }
        
        if (!m_oscSender->initialize()) {
            LOG_WARN("OSC Sender konnte nicht initialisiert werden - OSC deaktiviert");
        } else {
            LOG_INFO("OSC aktiv mit " + std::to_string(m_oscSender->getTargetCount()) + " Ziel(en)");
        }
        
        // OSC Receiver für externe Beat-Clock initialisieren
        int oscReceivePort = env.getInt("OSC_RECEIVE_PORT", 7775);
        m_oscReceiver = std::make_unique<OscReceiver>();
        m_oscReceiver->setPort(oscReceivePort);
        m_oscReceiver->setBeatClockPath("/beat");
        
        // Callback für externe Beat-Clock
        m_oscReceiver->setCallback([this](const ReceivedBeatClock& clock) {
            m_lastExternalBeatTime = clock.timestamp;
            if (clock.bpm > 0) {
                m_externalBpm = clock.bpm;
            }
            // Beat-Nummer speichern (1-4)
            if (clock.beatNumber >= 1 && clock.beatNumber <= 4) {
                m_externalBeatNumber.store(clock.beatNumber);
            }
            // Im Training-Modus (0): externe /beat direkt weitersenden
            if (m_clockMode.load() == 0 && clock.beatNumber >= 1 && clock.beatNumber <= 4) {
                // Interne States synchronisieren
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                        m_bpmTrackStates[ch].beatNumber = clock.beatNumber;
                    }
                }
                // Externe /beat direkt als eigene /beat weiterleiten
                if (m_oscSender && m_oscSender->isConnected()) {
                    BeatClockMessage msg;
                    msg.track_id = 0;
                    msg.beat_number = clock.beatNumber;
                    msg.bar_number = clock.bar;
                    msg.bpm = clock.bpm;
                    m_oscSender->sendBeatClock(msg);
                }
                LOG_DEBUG("Ext beat forwarded: " + std::to_string(clock.beatNumber) +
                         " bpm=" + std::to_string(clock.bpm));
            }
        });
        
        // Callback für Clock-Modus Wechsel
        m_oscReceiver->setClockModeCallback([this](int mode) {
            m_clockMode.store(mode);
            LOG_INFO("Clock-Modus via OSC: " + std::string(mode ? "EIGENE BEATCLOCK" : "TRAINING"));
        });
        
        // Callback für Tap (Downbeat setzen)
        m_oscReceiver->setTapCallback([this](int beat) {
            std::lock_guard<std::mutex> lock(m_mutex);
            for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                m_bpmTrackStates[ch].beatNumber = beat;
            }
            LOG_INFO("TAP: Beat auf " + std::to_string(beat) + " gesetzt");
        });
        
        if (m_oscReceiver->start()) {
            LOG_INFO("OSC Receiver gestartet auf Port " + std::to_string(oscReceivePort));
        }
        
        // Audio-Callback setzen
        m_jackClient->setMonoProcessCallback(
            [this](const std::vector<const CSAMPLE*>& bpmBuffers,
                   const std::vector<const CSAMPLE*>& vuBuffers,
                   int frames) {
                this->processAudio(bpmBuffers, vuBuffers, frames);
            });
        
        // JACK aktivieren
        if (!m_jackClient->activate()) {
            LOG_ERROR("JACK Client konnte nicht aktiviert werden");
            return false;
        }
        
        // Verfügbare Ports anzeigen
        auto ports = m_jackClient->getAvailablePorts();
        LOG_INFO("Verfügbare JACK-Ports:");
        for (const auto& port : ports) {
            LOG_INFO("  - " + port);
        }
        
        LOG_INFO("Beat Analyzer erfolgreich initialisiert");
        LOG_INFO("Sample Rate: " + std::to_string(m_jackClient->getSampleRate().value) + " Hz");
        LOG_INFO("Buffer Size: " + std::to_string(m_jackClient->getBufferSize()) + " Frames");
        
        return true;
    }
    
    bool run() {
        LOG_INFO("Beat Analyzer läuft (Ctrl+C zum Beenden)");
        LOG_INFO("Warte auf Audio...");
        
        auto lastStatusTime = std::chrono::steady_clock::now();
        auto lastVuOscTime = std::chrono::steady_clock::now();
        
        // VU OSC Rate: 25 Hz = 40ms (wie SuperCollider replyRate)
        const auto vuOscInterval = std::chrono::milliseconds(40);
        
        while (g_running) {
            auto now = std::chrono::steady_clock::now();
            
            // VU-Meter OSC senden (25 Hz, außerhalb des Audio-Callbacks)
            if (now - lastVuOscTime >= vuOscInterval) {
                sendVuMeterOsc();
                lastVuOscTime = now;
            }
            
            // Periodische Status-Ausgabe (alle 5 Sekunden)
            if (std::chrono::duration_cast<std::chrono::seconds>(now - lastStatusTime).count() >= 5) {
                printStatus();
                lastStatusTime = now;
            }
            
            // Kurzes Sleep um CPU zu schonen, aber schnell genug für 25 Hz
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        
        return true;
    }
    
    void shutdown() {
        LOG_INFO("Beat Analyzer wird beendet...");
        
        // Finale Beat-Info ausgeben
        for (int i = 0; i < m_numBpmChannels; ++i) {
            BeatInfo info = m_beatTrackers[i]->finalize();
            if (info.valid) {
                LOG_INFO("BPM Kanal " + std::to_string(i+1) + " - BPM: " + 
                        std::to_string(info.bpm) + " (" + 
                        std::to_string(info.beatFrames.size()) + " Beats erkannt)");
            }
        }
        
        if (m_oscReceiver) {
            m_oscReceiver->stop();
        }
        
        if (m_jackClient) {
            m_jackClient->deactivate();
        }
        
        LOG_INFO("Beat Analyzer beendet");
    }
    
private:
    // Track-Status für BPM Kanäle
    struct BpmTrackState {
        int64_t lastBeatFrame = 0;
        int64_t lastRealBeatFrame = 0;      // Letzter echter erkannter Beat
        int64_t prevRealBeatFrame = 0;      // Vorletzter echter Beat (für BPM-Berechnung)
        int64_t realBeatCount = 0;          // Anzahl echter Beats in Folge
        int beatNumber = 1;                 // 1-4 (Schlag im Takt)
        int barNumber = 1;                  // Takt-Nummer (fortlaufend)
        double currentBpm = 0.0;
        double realBeatBpm = 0.0;           // BPM basierend auf echten Beat-Abständen
        double recentBeatIntervals[4] = {0}; // Letzte 4 Beat-Intervalle für Toleranz-Check
        int intervalIndex = 0;
        bool hasSignal = false;
        bool usingRealBeats = false;        // true = echte Beats, false = synthetisch
    };
    
    // Konstanten für Beat-Modus-Wechsel
    static constexpr int REAL_BEATS_THRESHOLD = 4;      // So viele echte Beats für Umschaltung
    static constexpr double BEAT_TIMEOUT_FACTOR = 2.5;  // Nach X Beat-Intervallen auf synthetisch
    static constexpr double BEAT_TOLERANCE = 0.15;      // ±15% Toleranz für regelmäßige Beats
    
    // Track-Status für VU Kanäle
    struct VuTrackState {
        bool hasSignal = false;
    };
    
    // Prüft ob ein neues Beat-Intervall regelmäßig ist (innerhalb ±15% der letzten Intervalle)
    bool isBeatRegular(BpmTrackState& state, double newInterval) {
        // Speichere Intervall im Ringpuffer
        int idx = state.intervalIndex % 4;
        double oldInterval = state.recentBeatIntervals[idx];
        state.recentBeatIntervals[idx] = newInterval;
        state.intervalIndex++;
        
        // Brauchen mindestens 2 Intervalle zum Vergleich
        if (state.intervalIndex < 2) return true;
        
        // Berechne Durchschnitt der vorherigen Intervalle
        double sum = 0;
        int count = 0;
        for (int i = 0; i < 4; ++i) {
            if (state.recentBeatIntervals[i] > 0 && i != idx) {
                sum += state.recentBeatIntervals[i];
                count++;
            }
        }
        
        if (count == 0) return true;
        double avgInterval = sum / count;
        
        // Prüfe ob innerhalb ±15% Toleranz
        double tolerance = avgInterval * BEAT_TOLERANCE;
        return (newInterval >= avgInterval - tolerance && 
                newInterval <= avgInterval + tolerance);
    }
    
    void processAudio(const std::vector<const CSAMPLE*>& bpmBuffers,
                      const std::vector<const CSAMPLE*>& vuBuffers,
                      int frameCount) {
        
        // VU Kanäle verarbeiten - KEIN LOCK (lock-free, nur float writes)
        for (int ch = 0; ch < m_numVuChannels && ch < static_cast<int>(vuBuffers.size()); ++ch) {
            const CSAMPLE* buffer = vuBuffers[ch];
            m_vuMeters[ch]->processMono(buffer, frameCount);
        }
        
        // BPM Kanäle mit Lock (komplexere Datenstrukturen)
        std::lock_guard<std::mutex> lock(m_mutex);
        
        m_frameCount += frameCount;
        
        // Im Training-Modus (0): Keine eigene Beat-Generierung, externe /beat wird durchgereicht
        if (m_clockMode.load() == 0) return;
        
        int sampleRate = m_jackClient ? m_jackClient->getSampleRate().value : 44100;
        
        // BPM Kanäle verarbeiten
        for (int ch = 0; ch < m_numBpmChannels && ch < static_cast<int>(bpmBuffers.size()); ++ch) {
            const CSAMPLE* buffer = bpmBuffers[ch];
            
            // RMS berechnen für Signal-Check
            float sumSquares = 0.0f;
            for (int i = 0; i < frameCount; ++i) {
                sumSquares += buffer[i] * buffer[i];
            }
            float rms = std::sqrt(sumSquares / frameCount);
            m_bpmTrackStates[ch].hasSignal = (rms > 0.001f);  // ~-60dB
            
            // Beat Detection nur wenn Signal vorhanden
            if (m_bpmTrackStates[ch].hasSignal) {
                // Mono als Fake-Stereo für Beat Tracker
                std::vector<float> stereoBuffer(frameCount * 2);
                for (int i = 0; i < frameCount; ++i) {
                    stereoBuffer[i * 2] = buffer[i];
                    stereoBuffer[i * 2 + 1] = buffer[i];
                }
                m_beatTrackers[ch]->processAudio(stereoBuffer.data(), frameCount);
                
                double bpm = m_beatTrackers[ch]->getCurrentBpm();
                if (bpm > 0) {
                    m_bpmTrackStates[ch].currentBpm = bpm;
                }
                
                // Echter Beat erkannt?
                bool realBeatDetected = m_beatTrackers[ch]->hasBeatOccurred();
                
                if (realBeatDetected) {
                    // BPM aus echten Beat-Abständen berechnen (zuverlässiger!)
                    int64_t beatInterval = 0;
                    bool beatIsRegular = true;
                    
                    if (m_bpmTrackStates[ch].lastRealBeatFrame > 0) {
                        beatInterval = m_frameCount - m_bpmTrackStates[ch].lastRealBeatFrame;
                        if (beatInterval > 0) {
                            double measuredBpm = 60.0 * sampleRate / beatInterval;
                            // Sanity check: nur BPM innerhalb konfigurierter Limits
                            if (measuredBpm >= m_bpmMin && measuredBpm <= m_bpmMax) {
                                // Prüfe ob Beat regelmäßig ist (±15% Toleranz)
                                beatIsRegular = isBeatRegular(m_bpmTrackStates[ch], 
                                                              static_cast<double>(beatInterval));
                                
                                if (beatIsRegular) {
                                    // Glättung mit vorherigem Wert
                                    if (m_bpmTrackStates[ch].realBeatBpm > 0) {
                                        m_bpmTrackStates[ch].realBeatBpm = 
                                            m_bpmTrackStates[ch].realBeatBpm * 0.7 + measuredBpm * 0.3;
                                    } else {
                                        m_bpmTrackStates[ch].realBeatBpm = measuredBpm;
                                    }
                                    // Überschreibe das analysierte BPM mit dem gemessenen
                                    m_bpmTrackStates[ch].currentBpm = m_bpmTrackStates[ch].realBeatBpm;
                                }
                            } else {
                                beatIsRegular = false;  // Unplausible BPM = unregelmäßig
                            }
                        }
                    }
                    
                    // Nur regelmäßige Beats zählen
                    if (beatIsRegular) {
                        m_bpmTrackStates[ch].prevRealBeatFrame = m_bpmTrackStates[ch].lastRealBeatFrame;
                        m_bpmTrackStates[ch].lastRealBeatFrame = m_frameCount;
                        
                        // Echter Beat - Zähler erhöhen
                        m_bpmTrackStates[ch].realBeatCount++;
                        
                        // Nach genug echten Beats in Folge: auf echte Beats umschalten
                        if (m_bpmTrackStates[ch].realBeatCount >= REAL_BEATS_THRESHOLD) {
                            m_bpmTrackStates[ch].usingRealBeats = true;
                        }
                        
                        // Wenn wir echte Beats nutzen: diesen Beat senden
                        if (m_bpmTrackStates[ch].usingRealBeats) {
                            m_bpmTrackStates[ch].beatNumber = (m_bpmTrackStates[ch].beatNumber % 4) + 1;
                            m_bpmTrackStates[ch].lastBeatFrame = m_frameCount;
                            sendBeatClockForChannel(ch, true);  // true = echter Beat
                        }
                    } else {
                        // Unregelmäßiger Beat - Reset Zähler
                        m_bpmTrackStates[ch].realBeatCount = 0;
                    }
                }
                
                // Synthetische Clock wenn keine echten Beats
                // Bevorzuge gemessenes BPM aus echten Beats, falls vorhanden
                double effectiveBpm = m_bpmTrackStates[ch].realBeatBpm > 0 
                    ? m_bpmTrackStates[ch].realBeatBpm 
                    : m_bpmTrackStates[ch].currentBpm;
                
                if (effectiveBpm > 0) {
                    double framesPerBeat = (60.0 / effectiveBpm) * sampleRate;
                    
                    // Prüfe ob zu lange kein echter Beat kam -> auf synthetisch wechseln
                    int64_t framesSinceRealBeat = m_frameCount - m_bpmTrackStates[ch].lastRealBeatFrame;
                    if (framesSinceRealBeat > static_cast<int64_t>(framesPerBeat * BEAT_TIMEOUT_FACTOR)) {
                        if (m_bpmTrackStates[ch].usingRealBeats) {
                            m_bpmTrackStates[ch].usingRealBeats = false;
                            m_bpmTrackStates[ch].realBeatCount = 0;
                        }
                    }
                    
                    // Synthetische Beats nur wenn nicht im Real-Beat-Modus
                    if (!m_bpmTrackStates[ch].usingRealBeats) {
                        int64_t beatsSinceLast = (m_frameCount - m_bpmTrackStates[ch].lastBeatFrame)
                                                / static_cast<int64_t>(framesPerBeat);
                        
                        if (beatsSinceLast > 0) {
                            int oldBeat = m_bpmTrackStates[ch].beatNumber;
                            // Addiere beatsSinceLast, wrappe auf 1-4
                            m_bpmTrackStates[ch].beatNumber = ((oldBeat - 1 + beatsSinceLast) % 4) + 1;
                            m_bpmTrackStates[ch].lastBeatFrame = m_frameCount;
                            sendBeatClockForChannel(ch, false);  // false = synthetisch
                        }
                    }
                }
            }
        }
    }
    
    void sendBeatClockForChannel(int ch, bool isRealBeat = false) {
        if (!m_enableBeatclock) return;
        if (!m_oscSender || !m_oscSender->isConnected()) return;
        if (!m_bpmTrackStates[ch].hasSignal) return;
        
        // Im Training-Modus (0): Externe Clock als BPM-Referenz nutzen
        float effectiveBpm;
        if (m_clockMode.load() == 0 && m_externalBpm > 0) {
            effectiveBpm = m_externalBpm;
        } else {
            effectiveBpm = static_cast<float>(m_bpmTrackStates[ch].realBeatBpm > 0 
                ? m_bpmTrackStates[ch].realBeatBpm 
                : m_bpmTrackStates[ch].currentBpm);
        }
        
        // /beat iii  beat(1-4), bar, bpm
        BeatClockMessage msg;
        msg.track_id = ch;
        msg.beat_number = m_bpmTrackStates[ch].beatNumber;
        msg.bar_number = m_bpmTrackStates[ch].barNumber;
        msg.bpm = static_cast<int>(effectiveBpm + 0.5f);  // Runden
        
        m_oscSender->sendBeatClock(msg);
        
        // Debug-Log (nur gelegentlich)
        static int debugCounter = 0;
        if (++debugCounter % 16 == 0) {
            std::string modeStr = m_clockMode.load() == 0 ? " [TRAINING]" : "";
            LOG_DEBUG("Beat Ch" + std::to_string(ch+1) + 
                     (isRealBeat ? " [REAL]" : " [SYNTH]") + modeStr +
                     " beat=" + std::to_string(msg.beat_number) +
                     " bar=" + std::to_string(msg.bar_number) +
                     " BPM=" + std::to_string(msg.bpm));
        }
    }
    

    
    void sendVuMeterOsc() {
        if (!m_enableVu) return;
        // Sendet /vu/0-N mit [peak, rms] als lineare Werte (0.0-1.0) wie SuperCollider
        if (!m_oscSender || !m_oscSender->isConnected()) return;
        
        for (int ch = 0; ch < m_numVuChannels; ++ch) {
            // Immer senden, auch ohne Signal (wie SuperCollider)
            float rmsLinear = m_vuMeters[ch]->getRmsLinear();
            float peakLinear = m_vuMeters[ch]->getPeakLinear();
            
            // Sende /vu/0, /vu/1, etc. mit [peak, rms] als lineare Werte (wie SuperCollider)
            std::string vuPath = "/vu/" + std::to_string(ch);
            m_oscSender->sendFloats(vuPath, peakLinear, rmsLinear);
        }
    }
    
    void printStatus() {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        double seconds = static_cast<double>(m_frameCount) / 44100.0;
        std::string status = "Status: " + std::to_string(static_cast<int>(seconds)) + "s";
        
        bool anyBpmActive = false;
        for (int ch = 0; ch < m_numBpmChannels; ++ch) {
            if (m_bpmTrackStates[ch].hasSignal && m_bpmTrackStates[ch].currentBpm > 0) {
                status += " | BPM" + std::to_string(ch+1) + ": " + 
                         std::to_string(static_cast<int>(m_bpmTrackStates[ch].currentBpm));
                anyBpmActive = true;
            }
        }
        
        bool anyVuActive = false;
        // for (int ch = 0; ch < m_numVuChannels; ++ch) {
        //     if (m_vuTrackStates[ch].hasSignal) {
        //         anyVuActive = true;
        //     }
        // }
        
        if (!anyBpmActive && !anyVuActive) {
            status += " | Kein Signal";
        // } else if (anyVuActive) {
        //     status += " | VU aktiv";
        }
        
        if (m_oscSender && m_oscSender->isConnected()) {
            status += " | OSC: " + std::to_string(m_oscSender->getTargetCount()) + " Ziel(e)";
        }
        
        LOG_INFO(status);
    }
    
    // Audio
    std::shared_ptr<JackClient> m_jackClient;
    int m_numBpmChannels;
    int m_numVuChannels;
    
    // Beat Detection (nur für BPM Kanäle)
    std::vector<std::unique_ptr<RealTimeBeatTracker>> m_beatTrackers;
    std::vector<BpmTrackState> m_bpmTrackStates;
    
    // VU-Meter (nur für VU Kanäle)
    std::vector<std::unique_ptr<VuMeter>> m_vuMeters;
    std::vector<VuTrackState> m_vuTrackStates;
    
    // OSC
    std::shared_ptr<OscSender> m_oscSender;
    std::unique_ptr<OscReceiver> m_oscReceiver;
    
    // Training-Modus
    std::atomic<int> m_clockMode{1};  // 0=Training, 1=Eigene Beatclock
    std::atomic<int> m_externalBeatNumber{0};  // Letzter externer Beat (1-4), 0=keiner
    int64_t m_lastExternalBeatTime = 0;
    float m_externalBpm = 0.0f;
    
    // Status
    std::mutex m_mutex;
    int64_t m_frameCount;
    
    // Feature Toggles
    bool m_enableBeatclock = true;
    bool m_enableVu = true;
    
    bool m_enableBeat = true;
    
    // VU-Meter Konfiguration
    float m_vuRmsAttack = 0.8f;
    float m_vuRmsRelease = 0.2f;
    float m_vuPeakFalloff = 20.0f;
    
    // Beat-Latenz-Kompensation
    float m_beatLatencyMs = 23.0f;
    int64_t m_beatLatencyFrames = 1014;  // ~23ms bei 44.1kHz
    
    // BPM-Limits aus Konfiguration
    float m_bpmMin = 60.0f;
    float m_bpmMax = 200.0f;
};

// ============================================================================
// Main
// ============================================================================

void printUsage(const char* programName) {
    std::cout << "\nBeat Analyzer - Echtzeit Beat Detection mit OSC Output\n\n";
    std::cout << "Verwendung: " << programName << "\n\n";
    std::cout << "Konfiguration via .env Datei:\n";
    std::cout << "  NUM_BPM_CHANNELS=4  Anzahl BPM-Eingänge (bpm_1 bis bpm_N)\n";
    std::cout << "  NUM_VU_CHANNELS=2   Anzahl VU-Eingänge (vu_1 bis vu_N)\n";
    std::cout << "\nOSC Ziele (beliebig viele):\n";
    std::cout << "  OSC_HOST_Name=host:port\n";
    std::cout << "  Beispiel: OSC_HOST_Protokol=127.0.0.1:9000\n";
    std::cout << "\nOSC Adressen:\n";
    std::cout << "  /beat           Beat Clock: iif beat(1-4), bar, bpm\n";
    std::cout << "  /rms/1-N        RMS Level (VU Kanäle)\n";
    std::cout << "  /peak/1-N       Peak Level (VU Kanäle)\n";
    std::cout << "\n";
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════╗\n";
    std::cout << "║          BEAT ANALYZER v1.2.0             ║\n";
    std::cout << "║  Echtzeit Beat Detection + OSC Output     ║\n";
    std::cout << "╚═══════════════════════════════════════════╝\n";
    std::cout << "\n";
    
    if (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
        printUsage(argv[0]);
        return 0;
    }
    
    BeatAnalyzerApp app;
    
    if (!app.initialize("")) {
        LOG_ERROR("Initialisierung fehlgeschlagen");
        return 1;
    }
    
    if (!app.run()) {
        LOG_ERROR("Ausführung fehlgeschlagen");
        return 1;
    }
    
    app.shutdown();
    
    return 0;
}
