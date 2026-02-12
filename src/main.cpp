/**
 * Beat Analyzer - Hauptanwendung
 * 
 * Eigenständige Anwendung für:
 * - Audio Input via JACK (separate BPM und VU Kanäle)
 * - Beat Detection (eigene Implementierung)
 * - OSC Output (Beat Clock + VU-Meter)
 * 
 * Keine Pioneer-Abhängigkeiten - komplett eigenständig!
 */

#include "audio/jack_client.h"
#include "analysis/btrack_wrapper.h"
#include "analysis/vu_meter.h"
#include "osc/osc_sender.h"
#include "osc/osc_receiver.h"
#include "osc/pioneer_receiver.h"
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
        
        // Debug: Konsolen-Output
        m_debugVuConsole = env.getInt("DEBUG_VU_CONSOLE", 0) != 0;
        m_debugBpmConsole = env.getInt("DEBUG_BPM_CONSOLE", 1) != 0;
        
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
        m_bpmMin = env.getFloat("BPM_MIN", 60.0f);
        m_bpmMax = env.getFloat("BPM_MAX", 200.0f);
        
        LOG_INFO("Features: Beatclock=" + std::string(m_enableBeatclock ? "ON" : "OFF") +
                 " VU=" + std::string(m_enableVu ? "ON" : "OFF") +
                 " Beat=" + std::string(m_enableBeat ? "ON" : "OFF") +
                 " BPM=" + std::to_string(static_cast<int>(m_bpmMin)) + 
                 "-" + std::to_string(static_cast<int>(m_bpmMax)));
        
        // VU-Meter für VU Kanäle
        for (int i = 0; i < m_numVuChannels; ++i) {
            auto vuMeter = std::make_unique<VuMeter>();
            vuMeter->setRmsAttack(m_vuRmsAttack);
            vuMeter->setRmsRelease(m_vuRmsRelease);
            vuMeter->setPeakFalloff(m_vuPeakFalloff);
            m_vuMeters.push_back(std::move(vuMeter));
            m_vuTrackStates.push_back(VuTrackState{});
            // Vorberechnete OSC-Pfade für lock-free VU-Sending
            m_vuOscPaths.push_back("/vu/" + std::to_string(i));
        }
        
        // JACK Client initialisieren — MUSS vor BTrack passieren!
        // BTrack braucht die JACK Buffer Size als HopSize.
        std::string jackName = env.getString("JACK_CLIENT_NAME", "beat-analyzer");
        m_jackClient = std::make_shared<JackClient>(jackName, m_numBpmChannels, m_numVuChannels);
        
        if (!m_jackClient->initialize()) {
            LOG_ERROR("JACK Client konnte nicht initialisiert werden");
            return false;
        }
        
        // Beat Tracker für BPM Kanäle
        // HopSize = JACK Buffer Size — genau wie Max External:
        //   btrack_dsp64: hopSize = maxvectorsize; frameSize = hopSize * 2;
        //   btrack_perform64: processAudioFrame() einmal pro Callback
        // JACK liefert pro Callback genau bufferSize Samples = genau ein Hop.
        int btHopSize = m_jackClient->getBufferSize();  // z.B. 128
        int btFrameSize = btHopSize * 2;                // z.B. 256
        int sampleRate = m_jackClient->getSampleRate().value;
        
        LOG_INFO("BTrack: HopSize=" + std::to_string(btHopSize) + 
                 " FrameSize=" + std::to_string(btFrameSize) +
                 " SampleRate=" + std::to_string(sampleRate) +
                 " (~" + std::to_string(static_cast<int>(1000.0 * btHopSize / sampleRate)) + "ms/hop)");
        
        for (int i = 0; i < m_numBpmChannels; ++i) {
            auto btrack = std::make_unique<BTrackWrapper>(btHopSize, btFrameSize);
            m_btrackDetectors.push_back(std::move(btrack));
            m_bpmTrackStates.push_back(BpmTrackState{});
        }
        
        LOG_INFO(std::to_string(m_numBpmChannels) + " Beat Tracker, " + 
                 std::to_string(m_numVuChannels) + " VU-Meter initialisiert");
        
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
        
        // ================================================================
        // OSC Receiver: zwei Quellen, ein Steuerkanal
        // ================================================================
        // Port 7775: a3motion_osc (Modus 0) — /beat, /clockmode, /tap
        // Port 7776: Pioneer      (Modus 2) — /beat
        // /clockmode und /tap kommen immer über Port 7775 (a3motion)
        
        int portA3motion = env.getInt("OSC_PORT_A3MOTION", 7775);
        
        // --- a3motion Receiver (Port 7775) ---
        m_oscReceiverA3motion = std::make_unique<OscReceiver>();
        m_oscReceiverA3motion->setPort(portA3motion);
        m_oscReceiverA3motion->setBeatClockPath("/beat");
        
        // /beat von a3motion: nur in Modus 0 weiterleiten (an alle AUSSER motion)
        m_oscReceiverA3motion->setCallback([this](const ReceivedBeatClock& clock) {
            if (m_clockMode.load() == 0 && clock.beatNumber >= 1 && clock.beatNumber <= 4) {
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                        m_bpmTrackStates[ch].beatNumber = clock.beatNumber;
                    }
                }
                if (m_oscSender && m_oscSender->isConnected()) {
                    BeatClockMessage msg;
                    msg.track_id = 0;
                    msg.beat_number = clock.beatNumber;
                    msg.bar_number = clock.bar;
                    msg.bpm = clock.bpm;
                    // An alle AUSSER motion (die sendet ja selbst)
                    m_oscSender->sendBeatClockExcept(msg, "motion");
                }
            }
        });
        
        // /clockmode i — Modus wechseln (0=a3motion, 1=intern, 2=pioneer)
        m_oscReceiverA3motion->setClockModeCallback([this](int mode) {
            if (mode < 0 || mode > 2) mode = 1;  // Fallback: intern
            m_clockMode.store(mode);
            const char* modeNames[] = {"a3motion", "intern", "pioneer"};
            LOG_INFO("Clock-Modus: " + std::to_string(mode) + " (" + modeNames[mode] + ")");
        });
        
        // /tap i — Beat setzen (funktioniert in allen Modi)
        m_oscReceiverA3motion->setTapCallback([this](int beat) {
            auto now = std::chrono::steady_clock::now();
            int w = m_tapQueueWrite.load(std::memory_order_relaxed);
            int next = (w + 1) & (TAP_QUEUE_SIZE - 1);
            if (next != m_tapQueueRead.load(std::memory_order_acquire)) {
                m_tapQueue[w] = {beat, now};
                m_tapQueueWrite.store(next, std::memory_order_release);
            }
            LOG_INFO("TAP: Beat " + std::to_string(beat));
        });
        
        if (m_oscReceiverA3motion->start()) {
            LOG_INFO("OSC Receiver a3motion auf Port " + std::to_string(portA3motion));
        }
        
        // --- Pioneer Pro DJ Link Receiver (Ports 50000/50001/50002) ---
        int pioneerDeviceNum = env.getInt("PIONEER_DEVICE_NUM", 7);
        m_pioneerReceiver = std::make_unique<PioneerReceiver>();
        m_pioneerReceiver->setDeviceNumber(static_cast<uint8_t>(pioneerDeviceNum));
        m_pioneerReceiver->setDeviceName("beat-analyzer");
        
        // Beat von Pioneer: nur in Modus 2 weiterleiten (an alle Ziele)
        // PioneerReceiver liefert nur Master-Beats (double BPM, beat_within_bar 1-4)
        m_pioneerReceiver->setCallback([this](const PioneerBeat& beat) {
            if (m_clockMode.load() == 2 && beat.beatWithinBar >= 1 && beat.beatWithinBar <= 4) {
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                        m_bpmTrackStates[ch].beatNumber = beat.beatWithinBar;
                    }
                }
                if (m_oscSender && m_oscSender->isConnected()) {
                    BeatClockMessage msg;
                    msg.track_id = 0;
                    msg.beat_number = beat.beatWithinBar;
                    msg.bar_number = 0;  // Pioneer zählt keine Takte fortlaufend
                    msg.bpm = beat.effectiveBpm;
                    m_oscSender->sendBeatClock(msg);
                }
                
                if (m_debugBpmConsole) {
                    static auto lastPioneerBeat = std::chrono::steady_clock::now();
                    auto now = std::chrono::steady_clock::now();
                    double deltaMs = std::chrono::duration<double, std::milli>(now - lastPioneerBeat).count();
                    lastPioneerBeat = now;
                    printf("PIONEER_BEAT %d | BPM %5.1f (track=%.1f pitch=%+.1f%%) | delta %5.0fms | dev=%d %s\n",
                           beat.beatWithinBar, beat.effectiveBpm, beat.trackBpm, beat.pitchPercent,
                           deltaMs, beat.deviceNumber, beat.isMaster ? "MASTER" : "");
                    fflush(stdout);
                }
            }
        });
        
        if (m_pioneerReceiver->start()) {
            LOG_INFO("Pioneer Pro DJ Link Receiver aktiv (Virtual CDJ #" + std::to_string(pioneerDeviceNum) + ")");
        } else {
            LOG_WARN("Pioneer Receiver konnte nicht gestartet werden (Ports 50000-50002 belegt?)");
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
        // Verwendet sleep_until für drift-freies, absolutes Timing
        // WICHTIG: sleep_until ZUERST, dann senden - für gleichmäßiges Timing
        std::thread vuThread([this]() {
            const auto vuOscInterval = std::chrono::microseconds(1000000 / m_oscSendRate);
            auto nextSendTime = std::chrono::steady_clock::now() + vuOscInterval;
            while (g_running) {
                std::this_thread::sleep_until(nextSendTime);
                sendVuMeterOsc();
                nextSendTime += vuOscInterval;
            }
        });
        
        // Beat-Processing Thread: Beat Detection + OSC Senden
        // Läuft NICHT im JACK-Callback - JACK kopiert nur Audio in Ringpuffer
        std::thread beatThread([this]() {
            processBeatThread();
        });
        
        // Main-Thread wartet
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        vuThread.join();
        beatThread.join();
        
        return true;
    }
    
    void shutdown() {
        LOG_INFO("Beat Analyzer wird beendet...");
        
        // Finale Beat-Info ausgeben
        for (int i = 0; i < m_numBpmChannels; ++i) {
            double finalBpm = m_btrackDetectors[i]->getBpm();
            if (finalBpm > 0) {
                LOG_INFO("BPM Kanal " + std::to_string(i+1) + " - Letztes BPM: " + 
                        std::to_string(static_cast<int>(finalBpm)));
            }
        }
        
        if (m_oscReceiverA3motion) {
            m_oscReceiverA3motion->stop();
        }
        if (m_pioneerReceiver) {
            m_pioneerReceiver->stop();
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
    // Track-Status für BPM Kanäle
    // ARCHITEKTUR: REALBEAT → GRID → SYNTHBEAT
    // - REALBEAT: FFT-basierte Beat-Erkennung (mit Kick-Filter)
    // - GRID: Berechnet aus 5s REALBEAT-Daten, liefert gridBpm + gridPhase
    // - SYNTHBEAT: Kontinuierliche Clock, wird durch GRID korrigiert
    // - TAP: Setzt Beat 1 + liefert Suchmuster für GRID
    struct BpmTrackState {
        // SYNTHBEAT Clock-State
        double lastBeatFrame = 0.0;          // Letzter ausgegebener SYNTHBEAT (double für präzise Akkumulation)
        double synthPhase = 0.0;             // Phase-Akkumulator (0.0-1.0, >=1.0 = Beat)
        int beatNumber = 1;                 // 1-4 (Schlag im Takt)
        int barNumber = 1;                  // Takt-Nummer (fortlaufend)
        
        // BPM
        double currentBpm = 120.0;          // Aktive BPM für SYNTHBEAT (Default-Start)
        double acfBpm = 0.0;                // BPM aus ACF/Viterbi (REALBEAT)
        
        // GRID State (aus REALBEATs berechnet)
        double gridBpm = 0.0;               // Grid-berechnetes BPM
        double gridPhase = 0.0;             // Grid-Phase (0.0-1.0)
        double gridConfidence = 0.0;        // Grid-Konfidenz (0.0-1.0)
        int64_t lastGridUpdateFrame = 0;    // Letztes Grid-Update
        
        // SOLL-GRID State (aus TAP/REALBEAT Matching)
        double sollGridBpm = 0.0;           // SOLL-GRID BPM (aus TAPs)
        double sollGridPhase = 0.0;         // SOLL-GRID Phase (0.0-1.0)
        double sollGridConfidence = 0.0;    // SOLL-GRID Konfidenz
        int64_t sollGridFrame = 0;          // Nächster SOLL-Beat-Frame
        
        // TAP Pattern
        double tapPatternBpm = 0.0;         // TAP-Muster als Suchvorgabe für GRID
        int64_t tapPatternValidUntil = 0;   // Frame bis TAP-Muster gültig ist
        double tapLockedBpm = 0.0;          // TAP-gesetztes BPM (GRID darf nicht überschreiben)
        bool autoMaskCalculated = false;    // Auto-Maske wurde bereits berechnet
        
        // REALBEAT Tracking
        int64_t lastRealBeatFrame = 0;      // Letzter erkannter REALBEAT
        
        // Legacy (für Kompatibilität)
        double recentBpmValues[8] = {0};    // Letzte 8 gemessene BPM-Werte
        int bpmValueIndex = 0;
        int bpmValueCount = 0;
        int consecutiveRegularBeats = 0;
        
        // Signal
        bool hasSignal = false;
        int64_t lastSignalFrame = 0;           // Letzter Frame mit Signal (für Pause-Erkennung)
        bool wasInPause = true;                 // War gerade in Pause (neuer Song?)
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
        
        // BPM Kanäle: Audio nur in Ringbuffer kopieren (billig: ~512 bytes memcpy)
        // BTrack läuft NICHT im JACK-Callback — zu teuer (FFT + Transzendente)!
        for (int ch = 0; ch < m_numBpmChannels && ch < static_cast<int>(bpmBuffers.size()); ++ch) {
            auto& ring = m_audioRing[ch];
            int w = ring.wpos.load(std::memory_order_relaxed);
            int next = (w + 1) & AUDIO_RING_MASK;
            if (next != ring.rpos.load(std::memory_order_acquire)) {
                auto& slot = ring.slots[w];
                std::memcpy(slot.data, bpmBuffers[ch], frameCount * sizeof(float));
                slot.frameCount = frameCount;
                ring.wpos.store(next, std::memory_order_release);
            }
            // Bei Ringbuffer-Overflow: Frame droppen (besser als XRun)
        }
        
        m_frameCount.fetch_add(frameCount, std::memory_order_relaxed);
    }
    
    // Beat-Processing Thread: BTrack + Synthclock + TAP + OSC
    // BTrack läuft HIER, nicht im JACK-Callback!
    // JACK kopiert Audio in lock-free Ringbuffer → dieser Thread pollt und verarbeitet.
    void processBeatThread() {
        const auto interval = std::chrono::microseconds(500);  // 2kHz polling
        int sampleRate = m_jackClient ? m_jackClient->getSampleRate().value : 44100;
        
        auto nextWakeTime = std::chrono::steady_clock::now();
        
        while (g_running) {
            // ========================================
            // BTrack: Audio aus Ringbuffer verarbeiten (ALLE Modi)
            // BTrack muss IMMER laufen, damit BPM-Tracking bereit ist
            // wenn man in Modus 1 wechselt.
            // ========================================
            for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                auto& ring = m_audioRing[ch];
                // Alle verfügbaren Frames verarbeiten
                while (true) {
                    int r = ring.rpos.load(std::memory_order_relaxed);
                    if (r == ring.wpos.load(std::memory_order_acquire)) break;
                    
                    auto& slot = ring.slots[r];
                    bool beat = m_btrackDetectors[ch]->processSamples(slot.data, slot.frameCount);
                    ring.rpos.store((r + 1) & AUDIO_RING_MASK, std::memory_order_release);
                    
                    if (beat) {
                        m_btrackBeatFlag[ch].store(true, std::memory_order_release);
                    }
                    m_btrackBpmValue[ch].store(m_btrackDetectors[ch]->getBpm(), std::memory_order_release);
                }
            }
            
            if (m_clockMode.load() != 1) {
                // Modus 0 (a3motion) und 2 (pioneer): externe Quelle,
                // Synthclock pausiert, /beat wird im Receiver-Callback gesendet
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                nextWakeTime = std::chrono::steady_clock::now();
                continue;
            }
            
            int64_t currentFrame = m_frameCount.load(std::memory_order_relaxed);
            
            // ========================================
            // TAP-Queue verarbeiten
            // ========================================
            while (true) {
            int r = m_tapQueueRead.load(std::memory_order_relaxed);
            if (r == m_tapQueueWrite.load(std::memory_order_acquire)) break;
            TapEvent tap = m_tapQueue[r];
            m_tapQueueRead.store((r + 1) & (TAP_QUEUE_SIZE - 1), std::memory_order_release);
            int tapBeat = tap.beat;
            {
                double tapDeltaMs = m_hasPrevTap 
                    ? std::chrono::duration<double, std::milli>(tap.timestamp - m_lastTapTime).count() 
                    : 0.0;
                
                bool isFirstTap = !m_hasPrevTap || tapDeltaMs > 2000.0;
                
                if (isFirstTap) {
                    m_tapIntervalCount = 0;
                    m_tapIntervalIndex = 0;
                    m_tapBpm = 0.0;
                    
                    // Erster TAP = Downbeat/1 definieren
                    for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                        m_bpmTrackStates[ch].beatNumber = 1;
                        m_bpmTrackStates[ch].synthPhase = 0.0;
                    }
                    sendBeatClockForChannel(0, false);
                    
                    printf("TAP 1 | (erster Tap — Downbeat/1 definiert)\n");
                    fflush(stdout);
                } else {
                    m_tapIntervals[m_tapIntervalIndex % 8] = tapDeltaMs;
                    m_tapIntervalIndex++;
                    if (m_tapIntervalCount < 8) m_tapIntervalCount++;
                    
                    if (m_tapIntervalCount >= 2) {
                        double sorted[16];
                        int count = std::min(m_tapIntervalCount, 8);
                        for (int k = 0; k < count; ++k)
                            sorted[k] = m_tapIntervals[k];
                        std::sort(sorted, sorted + count);
                        double medianInterval = sorted[count / 2];
                        m_tapBpm = 60000.0 / medianInterval;
                    } else {
                        m_tapBpm = 60000.0 / tapDeltaMs;
                    }
                    
                    if (m_tapBpm >= m_bpmMin && m_tapBpm <= m_bpmMax) {
                        // BTrack Tempo hard-locken
                        for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                            m_btrackDetectors[ch]->fixTempo(m_tapBpm);
                            // Synthclock BPM sofort setzen
                            m_bpmTrackStates[ch].currentBpm = m_tapBpm;
                        }
                        
                        // TAP = Phase-Reset auf 1
                        if (m_tapIntervalCount >= 2) {
                            for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                                m_bpmTrackStates[ch].beatNumber = 1;
                                m_bpmTrackStates[ch].synthPhase = 0.0;
                            }
                        }
                        
                        printf("TAP %d | delta %7.1fms | tapBPM %5.1f → fixTempo | taps=%d\n",
                               tapBeat, tapDeltaMs, m_tapBpm, m_tapIntervalCount);
                    } else {
                        printf("TAP %d | delta %7.1fms | tapBPM %5.1f (außerhalb %.0f-%.0f)\n",
                               tapBeat, tapDeltaMs, m_tapBpm, m_bpmMin, m_bpmMax);
                    }
                    fflush(stdout);
                }
                m_lastTapTime = tap.timestamp;
                m_hasPrevTap = true;
            }
            } // while (tap queue)
            
            // ========================================
            // BTrack Beat-Flags + BPM abholen
            // ========================================
            for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                // Beat-Flag vom JACK-Callback abholen
                bool beatOccurred = m_btrackBeatFlag[ch].exchange(false, std::memory_order_acq_rel);
                
                // BPM von BTrack holen
                double btrackBpm = m_btrackBpmValue[ch].load(std::memory_order_acquire);
                
                if (beatOccurred) {
                    // BTrack hat einen Beat erkannt — BPM übernehmen
                    // Oktav-Korrektur: BTrack arbeitet intern 80-160
                    double correctedBpm = btrackBpm;
                    while (correctedBpm > 140.0) correctedBpm *= 0.5;
                    while (correctedBpm < 60.0) correctedBpm *= 2.0;
                    
                    // TAP-Referenz für Oktav-Korrektur
                    double tapRef = m_tapBpm;
                    if (tapRef > 0 && m_tapIntervalCount >= 2) {
                        double ratio = correctedBpm / tapRef;
                        if (ratio > 1.4 && ratio < 2.2) correctedBpm *= 0.5;
                        else if (ratio > 0.45 && ratio < 0.7) correctedBpm *= 2.0;
                    }
                    
                    // BPM sanft updaten (Synthclock-BPM)
                    if (m_bpmTrackStates[ch].currentBpm <= 0) {
                        m_bpmTrackStates[ch].currentBpm = correctedBpm;
                    } else {
                        double diff = correctedBpm - m_bpmTrackStates[ch].currentBpm;
                        double maxChange = 0.5;  // Nicht zu langsam — BTrack weiß was es tut
                        if (diff > maxChange) diff = maxChange;
                        else if (diff < -maxChange) diff = -maxChange;
                        m_bpmTrackStates[ch].currentBpm += diff;
                    }
                    
                    if (m_debugBpmConsole && ch == 0) {
                        static auto lastBtrackBeatTime = std::chrono::steady_clock::now();
                        auto now = std::chrono::steady_clock::now();
                        double deltaMs = std::chrono::duration<double, std::milli>(now - lastBtrackBeatTime).count();
                        lastBtrackBeatTime = now;
                        printf("BTRACK_BEAT: delta=%5.0fms bpm_raw=%.1f bpm_corr=%.1f synth=%.1f\n",
                               deltaMs, btrackBpm, correctedBpm, m_bpmTrackStates[ch].currentBpm);
                        fflush(stdout);
                    }
                }
            }
            
            // ========================================
            // SYNTHBEAT CLOCK
            // ========================================
            // Die Synthclock tickt unabhängig von BTrack-Beats.
            // BTrack-Beats liefern nur BPM-Updates.
            // TAP setzt Phase + Beat 1.
            for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                if (m_bpmTrackStates[ch].currentBpm > 0) {
                    double samplesPerBeat = sampleRate * 60.0 / m_bpmTrackStates[ch].currentBpm;
                    int64_t lastFrame = m_lastProcessedFrame[ch];
                    int64_t elapsed = currentFrame - lastFrame;
                    if (elapsed < 0) elapsed = 0;
                    
                    m_bpmTrackStates[ch].synthPhase += static_cast<double>(elapsed) / samplesPerBeat;
                    m_lastProcessedFrame[ch] = currentFrame;
                    
                    if (m_bpmTrackStates[ch].synthPhase >= 1.0) {
                        m_bpmTrackStates[ch].synthPhase -= 1.0;
                        m_bpmTrackStates[ch].beatNumber = (m_bpmTrackStates[ch].beatNumber % 4) + 1;
                        
                        sendBeatClockForChannel(ch, false);
                    }
                } else {
                    m_lastProcessedFrame[ch] = currentFrame;
                }
            }
            
            std::this_thread::sleep_until(nextWakeTime += interval);
        }
    }
    
    void sendBeatClockForChannel(int ch, bool isRealBeat = false) {
        if (!m_enableBeatclock) return;
        if (!m_oscSender || !m_oscSender->isConnected()) return;
        
        // Nur bei Modus 1 (intern) senden
        // Bei Modus 0/2 wird /beat im Receiver-Callback gesendet
        if (m_clockMode.load() != 1) return;
        
        double effectiveBpm = m_bpmTrackStates[ch].currentBpm;
        
        // /beat iif  beat(1-4), bar, bpm
        BeatClockMessage msg;
        msg.track_id = ch;
        msg.beat_number = m_bpmTrackStates[ch].beatNumber;
        msg.bar_number = m_bpmTrackStates[ch].barNumber;
        msg.bpm = effectiveBpm;
        
        m_oscSender->sendBeatClock(msg);
        
        // Timing-Analyse: Jeden Beat mit Timestamp + Delta ausgeben
        if (m_debugBpmConsole) {
            static auto lastBeatTime = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            double deltaMs = std::chrono::duration<double, std::milli>(now - lastBeatTime).count();
            lastBeatTime = now;
            
            double expectedMs = (msg.bpm > 0) ? (60000.0 / msg.bpm) : 0.0;
            double jitterMs = (expectedMs > 0) ? (deltaMs - expectedMs) : 0.0;
            
            printf("BEAT %d | BPM %5.1f | delta %7.1fms | expected %7.1fms | jitter %+.1fms | %s\n",
                   msg.beat_number, msg.bpm, deltaMs, expectedMs, jitterMs,
                   isRealBeat ? "REAL" : "SYNTH");
            fflush(stdout);
        }
    }
    

    
    void sendVuMeterOsc() {
        if (!m_enableVu) return;
        if (!m_oscSender || !m_oscSender->isConnected()) return;
        
        // Alle VU-Werte sammeln und als ein OSC Bundle senden
        // 1 UDP-Paket statt 12 — atomar, effizient
        float peaks[MAX_VU_CHANNELS];
        float rms[MAX_VU_CHANNELS];
        
        for (int ch = 0; ch < m_numVuChannels; ++ch) {
            peaks[ch] = m_vuMeters[ch]->getPeakLinear();
            rms[ch] = m_vuMeters[ch]->getRmsLinear();
        }
        
        m_oscSender->sendVuBundle(m_vuOscPaths.data(), peaks, rms, m_numVuChannels);
        
        // DEBUG: /vu/3 auf Konsole (nur wenn aktiviert)
        if (m_debugVuConsole && m_numVuChannels > 3) {
            static auto lastVuTime = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            double deltaMs = std::chrono::duration<double, std::milli>(now - lastVuTime).count();
            lastVuTime = now;
            printf("/vu/3 | peak %.3f | rms %.3f | delta %6.1fms\n", peaks[3], rms[3], deltaMs);
            fflush(stdout);
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
    
    // Beat-Kommunikation JACK-Callback → Beat-Thread (lock-free)
    static constexpr int MAX_BPM_CHANNELS = 8;
    static constexpr int MAX_VU_CHANNELS = 16;
    std::atomic<bool> m_btrackBeatFlag[MAX_BPM_CHANNELS] = {};  // Beat erkannt
    std::atomic<double> m_btrackBpmValue[MAX_BPM_CHANNELS] = {};  // BPM von BTrack
    int64_t m_lastProcessedFrame[MAX_BPM_CHANNELS] = {};  // Für Synthclock Phase-Tracking
    
    // Lock-free Audio Ringbuffer: JACK-Callback → BTrack-Thread
    // JACK kopiert nur Audio hierhin (billig: memcpy 512 bytes).
    // BTrack-Thread pollt und macht die schwere FFT+Onset-Berechnung.
    static constexpr int AUDIO_RING_SIZE = 64;  // Slots (Zweierpotenz), ~190ms bei 128/44100
    static constexpr int AUDIO_RING_MASK = AUDIO_RING_SIZE - 1;
    static constexpr int MAX_FRAME_SIZE = 1024;  // Max JACK buffer size
    struct AudioSlot {
        float data[MAX_FRAME_SIZE];  // Audio-Daten (ein JACK-Buffer)
        int frameCount = 0;
    };
    // Pro BPM-Kanal ein eigener Ringbuffer (SPSC: JACK schreibt, BTrack-Thread liest)
    struct AudioRing {
        AudioSlot slots[AUDIO_RING_SIZE];
        alignas(64) std::atomic<int> wpos{0};
        alignas(64) std::atomic<int> rpos{0};
    };
    AudioRing m_audioRing[MAX_BPM_CHANNELS];
    
    // Beat Detection (nur für BPM Kanäle)
    std::vector<std::unique_ptr<BTrackWrapper>> m_btrackDetectors;
    std::vector<BpmTrackState> m_bpmTrackStates;
    
    // VU-Meter (nur für VU Kanäle)
    std::vector<std::unique_ptr<VuMeter>> m_vuMeters;
    std::vector<VuTrackState> m_vuTrackStates;
    std::vector<std::string> m_vuOscPaths;  // Vorberechnete OSC-Pfade: /vu/0, /vu/1, ...
    
    // OSC
    std::shared_ptr<OscSender> m_oscSender;
    std::unique_ptr<OscReceiver> m_oscReceiverA3motion;  // Port 7775: a3motion_osc
    std::unique_ptr<PioneerReceiver> m_pioneerReceiver;   // Pioneer Pro DJ Link (Ports 50000-50002)
    
    // Clock-Modus: 0=a3motion, 1=intern, 2=pioneer
    std::atomic<int> m_clockMode{1};  // Default: eigene Beatclock
    
    // Tap-Queue: SPSC Ringbuffer, Zeitstempel wird im OSC-Thread erfasst
    struct TapEvent {
        int beat;
        std::chrono::steady_clock::time_point timestamp;
    };
    static constexpr int TAP_QUEUE_SIZE = 16;  // Muss Zweierpotenz sein
    TapEvent m_tapQueue[TAP_QUEUE_SIZE];
    std::atomic<int> m_tapQueueWrite{0};
    std::atomic<int> m_tapQueueRead{0};
    
    // Tap-Timing (nur im Beat-Thread benutzt)
    std::chrono::steady_clock::time_point m_lastTapTime{};
    bool m_hasPrevTap = false;
    double m_tapBpm = 0.0;              // BPM aus Tap-Intervallen
    double m_tapIntervals[8] = {0};     // Letzte 8 Tap-Intervalle (ms)
    int m_tapIntervalIndex = 0;
    int m_tapIntervalCount = 0;
    
    // Status
    std::mutex m_mutex;  // Nur noch für printStatus und Tap-Callback
    std::atomic<int64_t> m_frameCount{0};
    
    // Feature Toggles
    bool m_enableBeatclock = true;
    bool m_enableVu = true;
    bool m_enableBeat = true;
    
    // Debug: Konsolen-Output
    bool m_debugVuConsole = false;
    bool m_debugBpmConsole = true;
    
    // VU-Meter Konfiguration
    float m_vuRmsAttack = 0.8f;
    float m_vuRmsRelease = 0.2f;
    float m_vuPeakFalloff = 20.0f;
    
    // OSC Sende-Rate
    int m_oscSendRate = 25;         // Hz
    int m_oscSendIntervalMs = 40;   // 1000/rate ms
    
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
