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
        
        // Anzahl Kanäle (0-8)
        m_numBpmChannels = std::max(0, std::min(8, env.getInt("NUM_BPM_CHANNELS", 4)));
        m_numVuChannels = std::max(0, std::min(8, env.getInt("NUM_VU_CHANNELS", 2)));
        
        LOG_INFO("BPM Kanäle: " + std::to_string(m_numBpmChannels) + 
                 ", VU Kanäle: " + std::to_string(m_numVuChannels));
        
        // Feature Toggles
        m_enableBeatclock = env.getInt("ENABLE_BEATCLOCK", 1) != 0;
        m_enableVuRms = env.getInt("ENABLE_VU_RMS", 1) != 0;
        m_enableVuPeak = env.getInt("ENABLE_VU_PEAK", 1) != 0;
        m_enableBeat = env.getInt("ENABLE_BEAT", 1) != 0;
        
        LOG_INFO("Features: Beatclock=" + std::string(m_enableBeatclock ? "ON" : "OFF") +
                 " RMS=" + std::string(m_enableVuRms ? "ON" : "OFF") +
                 " Peak=" + std::string(m_enableVuPeak ? "ON" : "OFF") +
                 " Beat=" + std::string(m_enableBeat ? "ON" : "OFF"));
        
        // Beat Detector Konfiguration
        BeatDetectorConfig beatConfig;
        beatConfig.sampleRate = 44100;
        beatConfig.frameSize = 1024;
        beatConfig.hopSize = 512;
        beatConfig.minBpm = env.getFloat("BPM_MIN", 60.0f);
        beatConfig.maxBpm = env.getFloat("BPM_MAX", 200.0f);
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
            m_vuMeters.push_back(std::make_unique<VuMeter>());
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
        
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Periodische Status-Ausgabe
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - lastStatusTime).count() >= 5) {
                printStatus();
                lastStatusTime = now;
            }
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
        
        if (m_jackClient) {
            m_jackClient->deactivate();
        }
        
        LOG_INFO("Beat Analyzer beendet");
    }
    
private:
    // Track-Status für BPM Kanäle
    struct BpmTrackState {
        int64_t lastBeatFrame = 0;
        int beatNumber = 0;
        double currentBpm = 0.0;
        bool hasSignal = false;
    };
    
    // Track-Status für VU Kanäle
    struct VuTrackState {
        bool hasSignal = false;
    };
    
    void processAudio(const std::vector<const CSAMPLE*>& bpmBuffers,
                      const std::vector<const CSAMPLE*>& vuBuffers,
                      int frameCount) {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        m_frameCount += frameCount;
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
                
                // Echter Beat erkannt?
                if (m_beatTrackers[ch]->hasBeatOccurred()) {
                    sendBeatForChannel(ch);
                }
                
                double bpm = m_beatTrackers[ch]->getCurrentBpm();
                if (bpm > 0) {
                    m_bpmTrackStates[ch].currentBpm = bpm;
                }
                
                // Beat Clock senden
                if (m_bpmTrackStates[ch].currentBpm > 0) {
                    double framesPerBeat = (60.0 / m_bpmTrackStates[ch].currentBpm) * sampleRate;
                    int64_t beatsSinceLast = (m_frameCount - m_bpmTrackStates[ch].lastBeatFrame)
                                            / static_cast<int64_t>(framesPerBeat);
                    
                    if (beatsSinceLast > 0) {
                        m_bpmTrackStates[ch].beatNumber = (m_bpmTrackStates[ch].beatNumber + beatsSinceLast) % 4;
                        m_bpmTrackStates[ch].lastBeatFrame = m_frameCount;
                        sendBeatClockForChannel(ch);
                    }
                }
            }
        }
        
        // VU Kanäle verarbeiten
        for (int ch = 0; ch < m_numVuChannels && ch < static_cast<int>(vuBuffers.size()); ++ch) {
            const CSAMPLE* buffer = vuBuffers[ch];
            
            // RMS berechnen für Signal-Check
            float sumSquares = 0.0f;
            for (int i = 0; i < frameCount; ++i) {
                sumSquares += buffer[i] * buffer[i];
            }
            float rms = std::sqrt(sumSquares / frameCount);
            m_vuTrackStates[ch].hasSignal = (rms > 0.001f);  // ~-60dB
            
            // VU-Meter (Mono als Fake-Stereo)
            std::vector<float> stereoBuffer(frameCount * 2);
            for (int i = 0; i < frameCount; ++i) {
                stereoBuffer[i * 2] = buffer[i];
                stereoBuffer[i * 2 + 1] = buffer[i];
            }
            m_vuMeters[ch]->process(stereoBuffer.data(), frameCount);
        }
        
        // VU-Meter OSC senden
        sendVuMeterOsc();
    }
    
    void sendBeatClockForChannel(int ch) {
        if (!m_enableBeatclock) return;
        if (!m_oscSender || !m_oscSender->isConnected()) return;
        if (!m_bpmTrackStates[ch].hasSignal) return;
        
        BeatClockMessage msg;
        msg.track_id = ch;
        msg.frame_position = m_frameCount;
        msg.bpm = static_cast<float>(m_bpmTrackStates[ch].currentBpm);
        msg.beat_number = m_bpmTrackStates[ch].beatNumber;
        msg.beat_strength = 1.0f;
        
        m_oscSender->sendBeatClock(msg);
    }
    
    void sendBeatForChannel(int ch) {
        if (!m_enableBeat) return;
        if (!m_oscSender || !m_oscSender->isConnected()) return;
        if (!m_bpmTrackStates[ch].hasSignal) return;
        
        // Sende echten erkannten Beat: /beat/1, /beat/2, etc.
        std::string path = "/beat/" + std::to_string(ch + 1);
        float bpm = static_cast<float>(m_bpmTrackStates[ch].currentBpm);
        m_oscSender->sendFloat(path, bpm);
    }
    
    void sendVuMeterOsc() {
        if (!m_oscSender || !m_oscSender->isConnected()) return;
        
        for (int ch = 0; ch < m_numVuChannels; ++ch) {
            if (!m_vuTrackStates[ch].hasSignal) continue;
            
            float rmsDb = m_vuMeters[ch]->getRmsDb();
            float peakDb = m_vuMeters[ch]->getPeakDb();
            
            if (m_enableVuRms) {
                std::string rmsPath = "/rms/" + std::to_string(ch + 1);
                m_oscSender->sendFloat(rmsPath, rmsDb);
            }
            
            if (m_enableVuPeak) {
                std::string peakPath = "/peak/" + std::to_string(ch + 1);
                m_oscSender->sendFloat(peakPath, peakDb);
            }
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
        for (int ch = 0; ch < m_numVuChannels; ++ch) {
            if (m_vuTrackStates[ch].hasSignal) {
                anyVuActive = true;
            }
        }
        
        if (!anyBpmActive && !anyVuActive) {
            status += " | Kein Signal";
        } else if (anyVuActive) {
            status += " | VU aktiv";
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
    
    // Status
    std::mutex m_mutex;
    int64_t m_frameCount;
    
    // Feature Toggles
    bool m_enableBeatclock = true;
    bool m_enableVuRms = true;
    bool m_enableVuPeak = true;
    bool m_enableBeat = true;
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
    std::cout << "  /beatclock/1-N  Beat Clock (BPM Kanäle)\n";
    std::cout << "  /beat/1-N       Echte Beats (BPM Kanäle)\n";
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
