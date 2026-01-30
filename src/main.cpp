/**
 * Beat Analyzer - Hauptanwendung
 * 
 * Eigenständige Anwendung für:
 * - Audio Input via JACK (4x Stereo)
 * - Beat Detection (eigene Implementierung)
 * - OSC Output (Beat Clock)
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
#include <array>
#include <cmath>

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
// Beat Analyzer Anwendung
// ============================================================================

class BeatAnalyzerApp {
public:
    BeatAnalyzerApp() 
        : m_frameCount(0) {
        // VU-Meter für jeden Stereo-Kanal initialisieren
        for (int i = 0; i < 4; ++i) {
            m_vuMeters.push_back(std::make_unique<VuMeter>());
        }
    }
    
    bool initialize(const std::string&) {
        LOG_INFO("Beat Analyzer wird initialisiert...");
        
        // .env Konfiguration laden
        auto& env = EnvConfig::instance();
        if (env.load(".env")) {
            LOG_INFO(".env Konfiguration geladen");
        } else if (env.load(".env.example")) {
            LOG_INFO(".env.example als Fallback geladen");
        }
        
        // Log-Level setzen
        int logLevel = env.getInt("LOG_LEVEL", 1);
        Logger::setLogLevel(static_cast<LogLevel>(logLevel));
        
        // Feature Toggles
        m_enableBeatclock = env.getInt("ENABLE_BEATCLOCK", 1) != 0;
        m_enableVuRms = env.getInt("ENABLE_VU_RMS", 1) != 0;
        m_enableVuPeak = env.getInt("ENABLE_VU_PEAK", 1) != 0;
        
        LOG_INFO("Features: Beatclock=" + std::string(m_enableBeatclock ? "ON" : "OFF") +
                 " RMS=" + std::string(m_enableVuRms ? "ON" : "OFF") +
                 " Peak=" + std::string(m_enableVuPeak ? "ON" : "OFF"));
        
        // Beat Detector Konfiguration
        BeatDetectorConfig beatConfig;
        beatConfig.sampleRate = 44100;
        beatConfig.frameSize = 1024;
        beatConfig.hopSize = 512;
        beatConfig.minBpm = env.getFloat("BPM_MIN", 60.0f);
        beatConfig.maxBpm = env.getFloat("BPM_MAX", 200.0f);
        beatConfig.defaultBpm = 120.0;
        
        // Beat Tracker für jeden Stereo-Kanal (4x)
        for (int i = 0; i < 4; ++i) {
            auto tracker = std::make_unique<RealTimeBeatTracker>(beatConfig);
            if (!tracker->initialize()) {
                LOG_ERROR("Beat Tracker " + std::to_string(i) + " konnte nicht initialisiert werden");
                return false;
            }
            m_beatTrackers.push_back(std::move(tracker));
        }
        
        LOG_INFO("4 Beat Tracker initialisiert (je Stereo-Kanal)");
        
        // JACK Client initialisieren
        std::string jackName = env.getString("JACK_CLIENT_NAME", "beat-analyzer");
        m_jackClient = std::make_shared<JackClient>(jackName);
        
        if (!m_jackClient->initialize()) {
            LOG_ERROR("JACK Client konnte nicht initialisiert werden");
            return false;
        }
        
        // OSC Sender initialisieren
        std::string oscHost = env.getString("OSC_HOST", "127.0.0.1");
        int oscPort = env.getInt("OSC_PORT", 9000);
        
        m_oscSender = std::make_shared<OscSender>(oscHost, oscPort);
        if (!m_oscSender->initialize()) {
            LOG_WARN("OSC Sender konnte nicht initialisiert werden - OSC deaktiviert");
        } else {
            LOG_INFO("OSC Sender aktiv: " + oscHost + ":" + std::to_string(oscPort));
        }
        
        // Nur Stereo-Callback setzen (processStereoAudio macht alles)
        m_jackClient->setStereoProcessCallback(
            [this](const std::vector<const CSAMPLE*>& stereoBuffers, int frames) {
                this->processStereoAudio(stereoBuffers, frames);
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
        for (size_t i = 0; i < m_beatTrackers.size(); ++i) {
            BeatInfo info = m_beatTrackers[i]->finalize();
            if (info.valid) {
                LOG_INFO("Track " + std::to_string(i+1) + " - BPM: " + 
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
    // Track-Status für jeden der 4 Stereo-Kanäle
    struct TrackState {
        int64_t lastBeatFrame = 0;
        int beatNumber = 0;
        double currentBpm = 0.0;
        bool hasSignal = false;
    };
    
    void processStereoAudio(const std::vector<const CSAMPLE*>& stereoBuffers, int frameCount) {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        m_frameCount += frameCount;
        int sampleRate = m_jackClient ? m_jackClient->getSampleRate().value : 44100;
        
        // stereoBuffers enthält 8 Mono-Kanäle (4 Stereo-Paare)
        // Wir verarbeiten sie als Stereo-Paare: 0+1, 2+3, 4+5, 6+7
        
        for (int track = 0; track < 4; ++track) {
            int leftIdx = track * 2;
            int rightIdx = track * 2 + 1;
            
            if (leftIdx >= static_cast<int>(stereoBuffers.size()) || 
                rightIdx >= static_cast<int>(stereoBuffers.size())) {
                continue;
            }
            
            // Stereo zu Mono mixen und RMS berechnen für Signal-Check
            std::vector<float> monoBuffer(frameCount);
            float sumSquares = 0.0f;
            
            for (int i = 0; i < frameCount; ++i) {
                float left = stereoBuffers[leftIdx][i];
                float right = stereoBuffers[rightIdx][i];
                monoBuffer[i] = (left + right) * 0.5f;
                sumSquares += monoBuffer[i] * monoBuffer[i];
            }
            
            // Signal vorhanden? (RMS > -60dB)
            float rms = std::sqrt(sumSquares / frameCount);
            m_trackStates[track].hasSignal = (rms > 0.001f);  // ~-60dB
            
            // VU-Meter verarbeiten (Interleaved Stereo)
            if (track < static_cast<int>(m_vuMeters.size())) {
                std::vector<float> interleavedStereo(frameCount * 2);
                for (int i = 0; i < frameCount; ++i) {
                    interleavedStereo[i * 2] = stereoBuffers[leftIdx][i];
                    interleavedStereo[i * 2 + 1] = stereoBuffers[rightIdx][i];
                }
                m_vuMeters[track]->process(interleavedStereo.data(), frameCount);
            }
            
            // Beat Detection nur wenn Signal vorhanden
            if (m_trackStates[track].hasSignal && track < static_cast<int>(m_beatTrackers.size())) {
                // Stereo-Buffer für Tracker (erwartet interleaved)
                std::vector<float> stereoForTracker(frameCount * 2);
                for (int i = 0; i < frameCount; ++i) {
                    stereoForTracker[i * 2] = monoBuffer[i];
                    stereoForTracker[i * 2 + 1] = monoBuffer[i];
                }
                
                m_beatTrackers[track]->processAudio(stereoForTracker.data(), frameCount);
                
                double bpm = m_beatTrackers[track]->getCurrentBpm();
                if (bpm > 0) {
                    m_trackStates[track].currentBpm = bpm;
                }
                
                // Beat Clock senden wenn Beat erkannt
                if (m_trackStates[track].currentBpm > 0) {
                    double framesPerBeat = (60.0 / m_trackStates[track].currentBpm) * sampleRate;
                    int64_t beatsSinceLast = (m_frameCount - m_trackStates[track].lastBeatFrame) 
                                             / static_cast<int64_t>(framesPerBeat);
                    
                    if (beatsSinceLast > 0) {
                        m_trackStates[track].beatNumber = (m_trackStates[track].beatNumber + beatsSinceLast) % 4;
                        m_trackStates[track].lastBeatFrame = m_frameCount;
                        
                        // Beat Clock für diesen Track senden
                        sendBeatClockForTrack(track);
                    }
                }
            }
        }
        
        // VU-Meter OSC senden (nur für Tracks mit Signal)
        sendVuMeterOsc();
    }
    
    void sendBeatClockForTrack(int track) {
        if (!m_enableBeatclock) return;
        if (!m_oscSender || !m_oscSender->isConnected()) return;
        if (!m_trackStates[track].hasSignal) return;
        
        BeatClockMessage msg;
        msg.track_id = track;
        msg.frame_position = m_frameCount;
        msg.bpm = static_cast<float>(m_trackStates[track].currentBpm);
        msg.beat_number = m_trackStates[track].beatNumber;
        msg.beat_strength = 1.0f;
        
        m_oscSender->sendBeatClock(msg);
    }
    
    void sendVuMeterOsc() {
        if (!m_oscSender || !m_oscSender->isConnected()) return;
        
        for (int track = 0; track < 4 && track < static_cast<int>(m_vuMeters.size()); ++track) {
            // Nur senden wenn Signal vorhanden
            if (!m_trackStates[track].hasSignal) continue;
            
            float rmsDb = m_vuMeters[track]->getRmsDb();
            float peakDb = m_vuMeters[track]->getPeakDb();
            
            if (m_enableVuRms) {
                std::string rmsPath = "/rms/" + std::to_string(track + 1);
                m_oscSender->sendFloat(rmsPath, rmsDb);
            }
            
            if (m_enableVuPeak) {
                std::string peakPath = "/peak/" + std::to_string(track + 1);
                m_oscSender->sendFloat(peakPath, peakDb);
            }
        }
    }
    
    void printStatus() {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        double seconds = static_cast<double>(m_frameCount) / 44100.0;
        std::string status = "Status: " + std::to_string(static_cast<int>(seconds)) + "s";
        
        // Zeige BPM für jeden aktiven Track
        bool anyActive = false;
        for (int t = 0; t < 4; ++t) {
            if (m_trackStates[t].hasSignal && m_trackStates[t].currentBpm > 0) {
                status += " | T" + std::to_string(t+1) + ": " + 
                         std::to_string(static_cast<int>(m_trackStates[t].currentBpm)) + "BPM";
                anyActive = true;
            }
        }
        
        if (!anyActive) {
            status += " | Kein Signal";
        }
        
        if (m_oscSender && m_oscSender->isConnected()) {
            status += " | OSC: aktiv";
        }
        
        LOG_INFO(status);
    }
    
    // Audio
    std::shared_ptr<JackClient> m_jackClient;
    
    // Beat Detection (4 Tracker für 4 Stereo-Kanäle)
    std::vector<std::unique_ptr<RealTimeBeatTracker>> m_beatTrackers;
    
    // VU-Meter (4x für 4 Stereo-Kanäle)
    std::vector<std::unique_ptr<VuMeter>> m_vuMeters;
    
    // Track-Status
    std::array<TrackState, 4> m_trackStates;
    
    // OSC
    std::shared_ptr<OscSender> m_oscSender;
    
    // Status
    std::mutex m_mutex;
    int64_t m_frameCount;
    
    // Feature Toggles
    bool m_enableBeatclock = true;
    bool m_enableVuRms = true;
    bool m_enableVuPeak = true;
};

// ============================================================================
// Main
// ============================================================================

void printUsage(const char* programName) {
    std::cout << "\nBeat Analyzer - Echtzeit Beat Detection mit OSC Output\n\n";
    std::cout << "Verwendung: " << programName << " [config_file]\n\n";
    std::cout << "Features:\n";
    std::cout << "  - Audio Input: JACK (4x Stereo = 8 Kanäle)\n";
    std::cout << "  - Beat Detection: Onset Detection + Tempo Tracking pro Track\n";
    std::cout << "  - OSC Output: Beat Clock + VU-Meter (nur bei Signal)\n";
    std::cout << "\n";
    std::cout << "OSC Adressen:\n";
    std::cout << "  /beatclock/1-4  Beat Clock pro Track\n";
    std::cout << "  /rms/1-4        RMS Level pro Track\n";
    std::cout << "  /peak/1-4       Peak Level pro Track\n";
    std::cout << "\n";
}

int main(int argc, char* argv[]) {
    // Signal-Handler registrieren
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    // Banner
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════╗\n";
    std::cout << "║          BEAT ANALYZER v1.0.0             ║\n";
    std::cout << "║  Echtzeit Beat Detection + OSC Output     ║\n";
    std::cout << "╚═══════════════════════════════════════════╝\n";
    std::cout << "\n";
    
    // Hilfe anzeigen?
    if (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
        printUsage(argv[0]);
        return 0;
    }
    
    // Konfigurationspfad
    std::string configPath = (argc > 1) ? argv[1] : "config/config.yaml";
    
    // Anwendung starten
    BeatAnalyzerApp app;
    
    if (!app.initialize(configPath)) {
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
