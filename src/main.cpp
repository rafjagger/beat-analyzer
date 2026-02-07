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
#include <array>

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
        
        // OSC Sende-Rate (Hz) - gilt für VU und andere periodische OSC-Nachrichten
        m_oscSendRate = env.getInt("OSC_SEND_RATE", 25);
        if (m_oscSendRate < 1) m_oscSendRate = 1;
        if (m_oscSendRate > 100) m_oscSendRate = 100;
        m_oscSendIntervalMs = 1000 / m_oscSendRate;
        LOG_INFO("OSC Sende-Rate: " + std::to_string(m_oscSendRate) + " Hz (" + std::to_string(m_oscSendIntervalMs) + "ms)");
        
        // Beat Detection Latenz-Kompensation
        // Latenz-Quellen bei frameSize=512, hopSize=256:
        //   FFT Analyse-Zentrum: frameSize/2 = 256 samples = 5.8ms
        //   Peak Detection Delay: 1 hop = 256 samples = 5.8ms
        //   Ringpuffer + Thread: ~6ms
        //   Gesamt: ~18ms
        m_beatLatencyMs = env.getFloat("BEAT_LATENCY_MS", 18.0f);
        m_beatLatencyFrames = static_cast<int64_t>(m_beatLatencyMs / 1000.0f * 44100.0f);  // samples
        
        LOG_INFO("Features: Beatclock=" + std::string(m_enableBeatclock ? "ON" : "OFF") +
                 " VU=" + std::string(m_enableVu ? "ON" : "OFF") +
                 " Beat=" + std::string(m_enableBeat ? "ON" : "OFF") +
                 " Latenz-Kompensation=" + std::to_string(static_cast<int>(m_beatLatencyMs)) + "ms");
        
        // Beat Detector Konfiguration
        BeatDetectorConfig beatConfig;
        beatConfig.sampleRate = 44100;
        beatConfig.frameSize = 512;   // Kleinere FFT für tightere Zeitauflösung (~5.8ms/hop)
        beatConfig.hopSize = 256;     // 256 samples = 5.8ms Zeitauflösung
        m_bpmMin = env.getFloat("BPM_MIN", 60.0f);
        m_bpmMax = env.getFloat("BPM_MAX", 200.0f);
        beatConfig.minBpm = m_bpmMin;
        beatConfig.maxBpm = m_bpmMax;
        beatConfig.defaultBpm = 120.0;
        
        // Auto Gain Control
        beatConfig.agcEnabled = env.getInt("AGC_ENABLED", 1) != 0;
        beatConfig.agcTargetPeak = env.getFloat("AGC_TARGET_PEAK", 0.7f);
        
        LOG_INFO("Beat Detection: FFT=" + std::to_string(beatConfig.frameSize) +
                 " Hop=" + std::to_string(beatConfig.hopSize) +
                 " AGC=" + std::string(beatConfig.agcEnabled ? "ON" : "OFF") +
                 " BPM=" + std::to_string(static_cast<int>(m_bpmMin)) + 
                 "-" + std::to_string(static_cast<int>(m_bpmMax)));
        
        // Beat Tracker für BPM Kanäle
        for (int i = 0; i < m_numBpmChannels; ++i) {
            m_bpmRingBuffers.push_back(std::make_unique<BpmRingBuffer>());
        }
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
        
        // Callback für externe Beat-Clock (/beat iii)
        m_oscReceiver->setCallback([this](const ReceivedBeatClock& clock) {
            m_lastExternalBeatTime = clock.timestamp;
            if (clock.bpm > 0) {
                m_externalBpm = clock.bpm;
            }
            if (clock.beatNumber >= 1 && clock.beatNumber <= 4) {
                m_externalBeatNumber.store(clock.beatNumber);
            }
            
            // EXT=0 (Training): externe /beat direkt weitersenden
            if (m_clockMode.load() == 0 && clock.beatNumber >= 1 && clock.beatNumber <= 4) {
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                        m_bpmTrackStates[ch].beatNumber = clock.beatNumber;
                    }
                }
                // Direkt als eigene /beat weiterleiten
                if (m_oscSender && m_oscSender->isConnected()) {
                    BeatClockMessage msg;
                    msg.track_id = 0;
                    msg.beat_number = clock.beatNumber;
                    msg.bar_number = clock.bar;
                    msg.bpm = clock.bpm;
                    m_oscSender->sendBeatClock(msg);
                }
            }
        });
        
        // Callback für Clock-Modus Wechsel (/clockmode i)
        m_oscReceiver->setClockModeCallback([this](int mode) {
            m_clockMode.store(mode);
            LOG_INFO("Clock-Modus via OSC: " + std::string(mode ? "EXT=1 EIGENE BEATCLOCK" : "EXT=0 TRAINING"));
        });
        
        // Callback für Tap (/tap i) - setzt Beat sofort (nur bei EXT=1)
        // Tap wird via atomarem Flag an den Beat-Thread übergeben (thread-safe)
        m_oscReceiver->setTapCallback([this](int beat) {
            if (m_clockMode.load() == 1) {
                // Atomares Flag setzen - Beat-Thread wird es beim nächsten Durchlauf abholen
                m_tapRequested.store(beat, std::memory_order_release);
                LOG_INFO("TAP: Beat " + std::to_string(beat) + " angefordert");
            }
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
        
        // VU-Meter Thread: sendet /vu OSC mit konfigurierbarer Rate (komplett unabhängig)
        std::thread vuThread([this]() {
            const auto vuOscInterval = std::chrono::milliseconds(m_oscSendIntervalMs);
            while (g_running) {
                sendVuMeterOsc();
                std::this_thread::sleep_for(vuOscInterval);
            }
        });
        
        // Beat-Processing Thread: Beat Detection + OSC Senden
        // Läuft NICHT im JACK-Callback - JACK kopiert nur Audio in Ringpuffer
        std::thread beatThread([this]() {
            processBeatThread();
        });
        
        // Status Thread
        std::thread statusThread([this]() {
            while (g_running) {
                printStatus();
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }
        });
        
        // Main-Thread wartet
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        vuThread.join();
        beatThread.join();
        statusThread.join();
        
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
        
        // OSC Sender-Thread sauber beenden
        if (m_oscSender) {
            m_oscSender->shutdown();
        }
        
        if (m_jackClient) {
            m_jackClient->deactivate();
        }
        
        LOG_INFO("Beat Analyzer beendet");
    }
    
private:
    // Track-Status für BPM Kanäle - VEREINFACHT
    // Eine Synth-Clock läuft immer. Echte Beat-Events synchronisieren nur die Phase.
    struct BpmTrackState {
        // Clock-State
        int64_t lastBeatFrame = 0;          // Letzter ausgegebener Beat (synth oder real)
        int beatNumber = 1;                 // 1-4 (Schlag im Takt)
        int barNumber = 1;                  // Takt-Nummer (fortlaufend)
        
        // BPM (primär aus ACF/Viterbi, sekundär aus Beat-Intervallen)
        double currentBpm = 0.0;            // Aktive BPM für die Clock
        double acfBpm = 0.0;               // BPM aus ACF/Viterbi-Analyse
        
        // Beat-Intervall-basierte Verfeinerung
        int64_t lastRealBeatFrame = 0;      // Letzter erkannter Beat-Event
        double recentBpmValues[8] = {0};    // Letzte 8 gemessene BPM-Werte
        int bpmValueIndex = 0;
        int bpmValueCount = 0;
        int consecutiveRegularBeats = 0;    // Aufeinanderfolgende regelmäßige Beats
        
        // Signal
        bool hasSignal = false;
    };
    
    // Konstanten
    static constexpr double BEAT_TOLERANCE = 0.20;      // ±20% Toleranz für regelmäßige Beats
    static constexpr int PHASE_LOCK_BEATS = 3;          // So viele regelmäßige Beats für Phase-Lock
    
    // Track-Status für VU Kanäle
    struct VuTrackState {
        bool hasSignal = false;
    };
    
    void processAudio(const std::vector<const CSAMPLE*>& bpmBuffers,
                      const std::vector<const CSAMPLE*>& vuBuffers,
                      int frameCount) {
        
        // VU Kanäle verarbeiten - KEIN LOCK (lock-free, nur float writes)
        for (int ch = 0; ch < m_numVuChannels && ch < static_cast<int>(vuBuffers.size()); ++ch) {
            const CSAMPLE* buffer = vuBuffers[ch];
            m_vuMeters[ch]->processMono(buffer, frameCount);
        }
        
        // BPM Audio in Ringpuffer kopieren (lock-free für JACK-Thread)
        // Der Beat-Processing-Thread liest diese Daten dann aus
        for (int ch = 0; ch < m_numBpmChannels && ch < static_cast<int>(bpmBuffers.size()); ++ch) {
            const CSAMPLE* buffer = bpmBuffers[ch];
            auto& rb = *m_bpmRingBuffers[ch];
            int writePos = rb.writePos.load(std::memory_order_relaxed);
            
            for (int i = 0; i < frameCount; ++i) {
                rb.buffer[writePos] = buffer[i];
                writePos = (writePos + 1) % BPM_RINGBUF_SIZE;
            }
            rb.writePos.store(writePos, std::memory_order_release);
        }
        
        m_frameCount.fetch_add(frameCount, std::memory_order_relaxed);
    }
    
    // Beat-Processing: läuft in eigenem Thread, NICHT im JACK-Callback!
    // ARCHITEKTUR: Eine Synth-Clock läuft IMMER mit dem aktuellen BPM.
    // Echte Beat-Events synchronisieren nur die Phase der Synth-Clock.
    // Tap setzt Beat-Nummer + Phase sofort via atomarem Flag.
    void processBeatThread() {
        const auto interval = std::chrono::microseconds(500);  // 2kHz für präzises Timing
        int sampleRate = m_jackClient ? m_jackClient->getSampleRate().value : 44100;
        
        // Lokaler Buffer für Audio-Daten aus dem Ringpuffer
        std::vector<CSAMPLE> localBuf(BPM_RINGBUF_SIZE);
        
        while (g_running) {
            // Im Training-Modus (0): Keine eigene Beat-Generierung
            if (m_clockMode.load() == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            
            int64_t currentFrame = m_frameCount.load(std::memory_order_relaxed);
            
            // Prüfe ob Tap angefordert wurde (aus OSC-Thread)
            int tapBeat = m_tapRequested.exchange(0, std::memory_order_acquire);
            if (tapBeat > 0) {
                for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                    m_bpmTrackStates[ch].beatNumber = tapBeat;
                    m_bpmTrackStates[ch].lastBeatFrame = currentFrame;
                }
                // Sofort senden
                sendBeatClockForChannel(0, false);
            }
            
            for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                auto& rb = *m_bpmRingBuffers[ch];
                int writePos = rb.writePos.load(std::memory_order_acquire);
                int readPos = rb.readPos.load(std::memory_order_relaxed);
                
                // Berechne verfügbare Samples
                int available = (writePos - readPos + BPM_RINGBUF_SIZE) % BPM_RINGBUF_SIZE;
                if (available == 0) goto synthClock;
                
                // Audio aus Ringpuffer lesen
                for (int i = 0; i < available; ++i) {
                    localBuf[i] = rb.buffer[(readPos + i) % BPM_RINGBUF_SIZE];
                }
                rb.readPos.store(writePos, std::memory_order_relaxed);
                
                {
                    // RMS berechnen für Signal-Check
                    float sumSquares = 0.0f;
                    for (int i = 0; i < available; ++i) {
                        sumSquares += localBuf[i] * localBuf[i];
                    }
                    float rms = std::sqrt(sumSquares / available);
                    m_bpmTrackStates[ch].hasSignal = (rms > 0.001f);
                }
                
                // Beat Detection nur bei Signal
                if (m_bpmTrackStates[ch].hasSignal) {
                    m_beatTrackers[ch]->processMonoAudio(localBuf.data(), available);
                    
                    // ACF/Viterbi BPM übernehmen (primäre BPM-Quelle)
                    double acfBpm = m_beatTrackers[ch]->getCurrentBpm();
                    if (acfBpm > 0) {
                        m_bpmTrackStates[ch].acfBpm = acfBpm;
                        m_bpmTrackStates[ch].currentBpm = acfBpm;
                    }
                    
                    // Echter Beat erkannt? → Phase-Sync
                    if (m_beatTrackers[ch]->hasBeatOccurred()) {
                        int64_t compensatedFrame = currentFrame - m_beatLatencyFrames;
                        
                        // BPM aus Beat-Intervall berechnen (sekundäre Quelle zur Verfeinerung)
                        if (m_bpmTrackStates[ch].lastRealBeatFrame > 0) {
                            int64_t beatInterval = compensatedFrame - m_bpmTrackStates[ch].lastRealBeatFrame;
                            if (beatInterval > 0) {
                                double measuredBpm = 60.0 * sampleRate / beatInterval;
                                
                                // Nur akzeptieren wenn im BPM-Bereich und nahe am ACF-BPM
                                if (measuredBpm >= m_bpmMin && measuredBpm <= m_bpmMax) {
                                    double acfDiff = m_bpmTrackStates[ch].currentBpm > 0 
                                        ? std::abs(measuredBpm - m_bpmTrackStates[ch].currentBpm) / m_bpmTrackStates[ch].currentBpm
                                        : 0.0;
                                    
                                    if (acfDiff < BEAT_TOLERANCE) {
                                        // Beat-Intervall passt zum ACF-BPM → BPM verfeinern
                                        int bpmIdx = m_bpmTrackStates[ch].bpmValueIndex % 8;
                                        m_bpmTrackStates[ch].recentBpmValues[bpmIdx] = measuredBpm;
                                        m_bpmTrackStates[ch].bpmValueIndex++;
                                        if (m_bpmTrackStates[ch].bpmValueCount < 8)
                                            m_bpmTrackStates[ch].bpmValueCount++;
                                        
                                        // Median berechnen
                                        int count = m_bpmTrackStates[ch].bpmValueCount;
                                        if (count >= 3) {
                                            std::array<double, 8> sorted{};
                                            for (int k = 0; k < count; ++k)
                                                sorted[static_cast<size_t>(k)] = m_bpmTrackStates[ch].recentBpmValues[k];
                                            std::sort(sorted.begin(), sorted.begin() + count);
                                            double medianBpm = sorted[static_cast<size_t>(count / 2)];
                                            // Verfeinerte BPM übernehmen (präziser als ACF allein)
                                            m_bpmTrackStates[ch].currentBpm = medianBpm;
                                        }
                                        
                                        m_bpmTrackStates[ch].consecutiveRegularBeats++;
                                    } else {
                                        m_bpmTrackStates[ch].consecutiveRegularBeats = 0;
                                    }
                                }
                            }
                        }
                        
                        m_bpmTrackStates[ch].lastRealBeatFrame = compensatedFrame;
                        
                        // Phase-Lock: Wenn genug regelmäßige Beats, synce die Clock-Phase
                        if (m_bpmTrackStates[ch].consecutiveRegularBeats >= PHASE_LOCK_BEATS) {
                            // Beat-Nummer weiterschalten
                            m_bpmTrackStates[ch].beatNumber = (m_bpmTrackStates[ch].beatNumber % 4) + 1;
                            // Phase der Synth-Clock auf den echten Beat setzen
                            m_bpmTrackStates[ch].lastBeatFrame = compensatedFrame;
                            sendBeatClockForChannel(ch, true);
                            // Goto überspringt Synth-Clock-Check für diesen Channel
                            continue;
                        }
                    }
                } // hasSignal
                
                synthClock:
                // === SYNTHETISCHE CLOCK (läuft IMMER) ===
                {
                    double bpm = m_bpmTrackStates[ch].currentBpm;
                    if (bpm > 0) {
                        double framesPerBeat = (60.0 / bpm) * sampleRate;
                        
                        // Initialisierung: wenn lastBeatFrame noch nie gesetzt wurde,
                        // oder wenn es zu weit in der Vergangenheit liegt (>2 Beats),
                        // auf jetzt setzen damit die Clock sofort startet.
                        int64_t elapsed = currentFrame - m_bpmTrackStates[ch].lastBeatFrame;
                        if (m_bpmTrackStates[ch].lastBeatFrame == 0 || elapsed > static_cast<int64_t>(framesPerBeat * 2.5)) {
                            m_bpmTrackStates[ch].lastBeatFrame = currentFrame;
                        }
                        
                        elapsed = currentFrame - m_bpmTrackStates[ch].lastBeatFrame;
                        if (elapsed >= static_cast<int64_t>(framesPerBeat)) {
                            m_bpmTrackStates[ch].beatNumber = (m_bpmTrackStates[ch].beatNumber % 4) + 1;
                            // Drift-freie Korrektur
                            m_bpmTrackStates[ch].lastBeatFrame += static_cast<int64_t>(framesPerBeat);
                            sendBeatClockForChannel(ch, false);
                        }
                    }
                }
            }
            
            std::this_thread::sleep_for(interval);
        }
    }
    
    void sendBeatClockForChannel(int ch, bool isRealBeat = false) {
        if (!m_enableBeatclock) return;
        if (!m_oscSender || !m_oscSender->isConnected()) return;
        
        // Nur bei EXT=1 (eigene Beatclock) senden
        // Bei EXT=0 wird extern empfangene /beat direkt weitergeleitet
        if (m_clockMode.load() == 0) return;
        
        float effectiveBpm = static_cast<float>(m_bpmTrackStates[ch].currentBpm);
        
        // /beat iii  beat(1-4), bar, bpm
        BeatClockMessage msg;
        msg.track_id = ch;
        msg.beat_number = m_bpmTrackStates[ch].beatNumber;
        msg.bar_number = m_bpmTrackStates[ch].barNumber;
        msg.bpm = static_cast<int>(effectiveBpm + 0.5f);
        
        m_oscSender->sendBeatClock(msg);
        
        // Debug-Log (nur gelegentlich)
        static int debugCounter = 0;
        if (++debugCounter % 16 == 0) {
            LOG_DEBUG("Beat Ch" + std::to_string(ch+1) + 
                     (isRealBeat ? " [REAL]" : " [SYNTH]") +
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
        double seconds = static_cast<double>(m_frameCount.load(std::memory_order_relaxed)) / 44100.0;
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
    
    // Lock-free Ringpuffer für BPM-Audio (JACK-Callback → Beat-Thread)
    static constexpr int BPM_RINGBUF_SIZE = 16384;  // ~370ms bei 44.1kHz
    struct BpmRingBuffer {
        CSAMPLE buffer[16384] = {};
        std::atomic<int> writePos{0};
        std::atomic<int> readPos{0};
    };
    std::vector<std::unique_ptr<BpmRingBuffer>> m_bpmRingBuffers;
    
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
    std::atomic<int> m_tapRequested{0};  // Tap-Anforderung aus OSC-Thread (0=keine, 1-4=Beat)
    int64_t m_lastExternalBeatTime = 0;
    float m_externalBpm = 0.0f;
    
    // Status
    std::mutex m_mutex;  // Nur noch für printStatus und Tap-Callback
    std::atomic<int64_t> m_frameCount{0};
    
    // Feature Toggles
    bool m_enableBeatclock = true;
    bool m_enableVu = true;
    
    bool m_enableBeat = true;
    
    // VU-Meter Konfiguration
    float m_vuRmsAttack = 0.8f;
    float m_vuRmsRelease = 0.2f;
    float m_vuPeakFalloff = 20.0f;
    
    // OSC Sende-Rate
    int m_oscSendRate = 25;         // Hz
    int m_oscSendIntervalMs = 40;   // 1000/rate ms
    
    // Beat-Latenz-Kompensation
    float m_beatLatencyMs = 18.0f;
    int64_t m_beatLatencyFrames = 794;  // ~18ms bei 44.1kHz
    
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
