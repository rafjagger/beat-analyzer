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
#include "osc/osc_sender.h"
#include "config/config_loader.h"
#include "util/logging.h"

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <mutex>

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
        : m_frameCount(0),
          m_lastBeatFrame(0),
          m_beatNumber(0) {
    }
    
    bool initialize(const std::string& configPath) {
        LOG_INFO("Beat Analyzer wird initialisiert...");
        
        // Konfiguration laden
        m_config = std::make_shared<ConfigLoader>(configPath);
        if (!m_config->load()) {
            LOG_WARN("Konfigurationsdatei nicht gefunden, nutze Standardwerte");
        }
        
        // Log-Level setzen
        int logLevel = m_config->getInt("logging.level", 1);
        Logger::setLogLevel(static_cast<LogLevel>(logLevel));
        
        // Beat Detector Konfiguration
        BeatDetectorConfig beatConfig;
        beatConfig.sampleRate = m_config->getInt("audio.sample_rate", 44100);
        beatConfig.frameSize = 1024;
        beatConfig.hopSize = 512;
        beatConfig.minBpm = m_config->getFloat("analysis.bpm_range_min", 60.0f);
        beatConfig.maxBpm = m_config->getFloat("analysis.bpm_range_max", 200.0f);
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
        std::string jackName = m_config->getString("jack.client_name", "beat-analyzer");
        m_jackClient = std::make_shared<JackClient>(jackName);
        
        if (!m_jackClient->initialize()) {
            LOG_ERROR("JACK Client konnte nicht initialisiert werden");
            return false;
        }
        
        // OSC Sender initialisieren
        std::string oscHost = m_config->getString("osc.host", "127.0.0.1");
        int oscPort = m_config->getInt("osc.port", 9000);
        
        m_oscSender = std::make_shared<OscSender>(oscHost, oscPort);
        if (!m_oscSender->initialize()) {
            LOG_WARN("OSC Sender konnte nicht initialisiert werden - OSC deaktiviert");
        } else {
            LOG_INFO("OSC Sender aktiv: " + oscHost + ":" + std::to_string(oscPort));
        }
        
        // JACK Process Callback setzen
        m_jackClient->setProcessCallback([this](const CSAMPLE* samples, int frames) {
            this->processAudio(samples, frames);
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
                LOG_INFO("Track " + std::to_string(i) + " - BPM: " + 
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
    void processAudio(const CSAMPLE* samples, int frameCount) {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        m_frameCount += frameCount;
        
        // Audio wird bereits als Mono geliefert vom JackClient
        // Hier verarbeiten wir es für jeden Track
        
        // Erstelle Stereo-Buffer aus Mono (für die Tracker API)
        std::vector<Sample> stereoBuffer(frameCount * 2);
        for (int i = 0; i < frameCount; ++i) {
            stereoBuffer[i * 2] = samples[i];
            stereoBuffer[i * 2 + 1] = samples[i];
        }
        
        // Verarbeite mit erstem Beat Tracker (alle 4 Tracker bekommen das gleiche Signal)
        // In einer echten Anwendung würden separate Kanäle genutzt
        if (!m_beatTrackers.empty()) {
            m_beatTrackers[0]->processAudio(stereoBuffer.data(), frameCount);
            
            double currentBpm = m_beatTrackers[0]->getCurrentBpm();
            if (currentBpm > 0) {
                m_currentBpm = currentBpm;
            }
        }
        
        // Echtzeit Beat-Clock berechnen
        if (m_currentBpm > 0) {
            // Frames pro Beat berechnen
            int sampleRate = m_jackClient ? m_jackClient->getSampleRate().value : 44100;
            double framesPerBeat = (60.0 / m_currentBpm) * sampleRate;
            
            // Prüfen ob wir einen Beat überschritten haben
            int64_t beatsSinceLast = (m_frameCount - m_lastBeatFrame) / static_cast<int64_t>(framesPerBeat);
            
            if (beatsSinceLast > 0) {
                m_beatNumber = (m_beatNumber + beatsSinceLast) % 4;
                m_lastBeatFrame = m_frameCount;
                
                // OSC senden
                sendBeatClock();
            }
        }
    }
    
    void sendBeatClock() {
        if (!m_oscSender || !m_oscSender->isConnected()) return;
        
        // Sende Beat Clock für alle 4 Tracks
        for (int track = 0; track < 4; ++track) {
            BeatClockMessage msg;
            msg.track_id = track;
            msg.frame_position = m_frameCount;
            msg.bpm = static_cast<float>(m_currentBpm);
            msg.beat_number = m_beatNumber;
            msg.beat_strength = 1.0f;
            
            m_oscSender->sendBeatClock(msg);
        }
    }
    
    void printStatus() {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        double seconds = static_cast<double>(m_frameCount) / 44100.0;
        
        std::string status = "Status: " + std::to_string(static_cast<int>(seconds)) + "s";
        
        if (m_currentBpm > 0) {
            status += " | BPM: " + std::to_string(static_cast<int>(m_currentBpm));
            status += " | Beat: " + std::to_string(m_beatNumber + 1) + "/4";
        } else {
            status += " | Analysiere...";
        }
        
        if (m_oscSender && m_oscSender->isConnected()) {
            status += " | OSC: aktiv";
        }
        
        LOG_INFO(status);
    }
    
    // Konfiguration
    std::shared_ptr<ConfigLoader> m_config;
    
    // Audio
    std::shared_ptr<JackClient> m_jackClient;
    
    // Beat Detection (4 Tracker für 4 Stereo-Kanäle)
    std::vector<std::unique_ptr<RealTimeBeatTracker>> m_beatTrackers;
    
    // OSC
    std::shared_ptr<OscSender> m_oscSender;
    
    // Status
    std::mutex m_mutex;
    int64_t m_frameCount;
    int64_t m_lastBeatFrame;
    int m_beatNumber;
    double m_currentBpm = 0.0;
};

// ============================================================================
// Main
// ============================================================================

void printUsage(const char* programName) {
    std::cout << "\nBeat Analyzer - Echtzeit Beat Detection mit OSC Output\n\n";
    std::cout << "Verwendung: " << programName << " [config_file]\n\n";
    std::cout << "Optionen:\n";
    std::cout << "  config_file    Pfad zur Konfigurationsdatei (Standard: config/config.yaml)\n";
    std::cout << "\n";
    std::cout << "Features:\n";
    std::cout << "  - Audio Input: JACK (4x Stereo = 8 Kanäle)\n";
    std::cout << "  - Beat Detection: Onset Detection + Tempo Tracking\n";
    std::cout << "  - OSC Output: Beat Clock an Pro Tools (4 Tracks)\n";
    std::cout << "\n";
    std::cout << "OSC Message Format:\n";
    std::cout << "  /beatclock [track_id] [frame_pos] [bpm] [beat_number] [strength]\n";
    std::cout << "\n";
}

int main(int argc, char* argv[]) {
    // Signal Handler registrieren
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    // Konfigurationspfad
    std::string configPath = "config/config.yaml";
    
    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }
        configPath = arg;
    }
    
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════╗\n";
    std::cout << "║          BEAT ANALYZER v1.0.0             ║\n";
    std::cout << "║  Echtzeit Beat Detection + OSC Output     ║\n";
    std::cout << "╚═══════════════════════════════════════════╝\n";
    std::cout << "\n";
    
    BeatAnalyzerApp app;
    
    if (!app.initialize(configPath)) {
        LOG_ERROR("Initialisierung fehlgeschlagen");
        return 1;
    }
    
    if (!app.run()) {
        LOG_ERROR("Laufzeitfehler");
        return 1;
    }
    
    app.shutdown();
    
    return 0;
}
