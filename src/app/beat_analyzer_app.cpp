/**
 * BeatAnalyzerApp — Konfiguration + Lifecycle
 *
 * initialize(): .env laden, JACK/BTrack/OSC/Pioneer aufsetzen
 * run():        Threads starten (VU, Beat), warten auf Ctrl+C
 * shutdown():   Alles sauber beenden
 */

#include "app/beat_analyzer_app.h"

#include <iostream>

using namespace BeatAnalyzer::Audio;
using namespace BeatAnalyzer::OSC;
using namespace BeatAnalyzer::Util;

namespace BeatAnalyzer {

// ============================================================================
// Helper
// ============================================================================

static bool parseHostPort(const std::string& value, std::string& host, int& port) {
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
// Konstruktor
// ============================================================================

BeatAnalyzerApp::BeatAnalyzerApp()
    : m_numBpmChannels(1)
    , m_numVuChannels(12)
    , m_frameCount(0)
{
}

// ============================================================================
// initialize()
// ============================================================================

bool BeatAnalyzerApp::initialize() {
    LOG_INFO("Beat Analyzer wird initialisiert...");
    
    // .env Konfiguration laden
    auto& env = EnvConfig::instance();
    if (env.load(".env") || env.load("../.env")) {
        LOG_INFO(".env Konfiguration geladen");
    } else if (env.load(".env.example")) {
        LOG_INFO(".env.example als Fallback geladen");
    }
    
    loadConfig(env);
    initVuMeters();
    
    // JACK Client initialisieren — MUSS vor BTrack passieren (braucht Buffer Size)
    std::string jackName = env.getString("JACK_CLIENT_NAME", "beat-analyzer");
    m_jackClient = std::make_shared<JackClient>(jackName, m_numBpmChannels, m_numVuChannels);
    
    if (!m_jackClient->initialize()) {
        LOG_ERROR("JACK Client konnte nicht initialisiert werden");
        return false;
    }
    
    // Beat Tracker: HopSize = JACK Buffer Size (wie Max External)
    int btHopSize = m_jackClient->getBufferSize();
    int btFrameSize = btHopSize * 2;
    int sampleRate = m_jackClient->getSampleRate().value;
    
    LOG_INFO("BTrack: HopSize=" + std::to_string(btHopSize) + 
             " FrameSize=" + std::to_string(btFrameSize) +
             " SampleRate=" + std::to_string(sampleRate) +
             " (~" + std::to_string(static_cast<int>(1000.0 * btHopSize / sampleRate)) + "ms/hop)");
    
    initBeatTrackers(btHopSize, btFrameSize);
    initOscSender(env);
    initOscReceiver(env);
    initPioneerReceiver(env);
    
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

// ============================================================================
// run()
// ============================================================================

bool BeatAnalyzerApp::run() {
    LOG_INFO("Beat Analyzer läuft (Ctrl+C zum Beenden)");
    LOG_INFO("Warte auf Audio...");
    
    // VU-Meter Thread: sendet /vu OSC mit konfigurierbarer Rate
    std::thread vuThread([this]() {
        const auto vuOscInterval = std::chrono::microseconds(1000000 / m_oscSendRate);
        auto nextSendTime = std::chrono::steady_clock::now() + vuOscInterval;
        while (g_running) {
            std::this_thread::sleep_until(nextSendTime);
            sendVuMeterOsc();
            nextSendTime += vuOscInterval;
        }
    });
    
    // Beat-Processing Thread: BTrack + Synthclock + TAP + OSC
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

// ============================================================================
// shutdown()
// ============================================================================

void BeatAnalyzerApp::shutdown() {
    LOG_INFO("Beat Analyzer wird beendet...");
    
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
    if (m_oscSender) {
        m_oscSender->shutdown();
    }
    if (m_jackClient) {
        m_jackClient->deactivate();
    }
    
    LOG_INFO("Beat Analyzer beendet");
}

// ============================================================================
// Konfiguration laden
// ============================================================================

void BeatAnalyzerApp::loadConfig(EnvConfig& env) {
    // Log-Level
    int logLevel = env.getInt("LOG_LEVEL", 1);
    Logger::setLogLevel(static_cast<LogLevel>(logLevel));
    
    // Kanäle
    m_numBpmChannels = std::max(0, env.getInt("NUM_BPM_CHANNELS", 1));
    m_numVuChannels = std::max(0, env.getInt("NUM_VU_CHANNELS", 12));
    
    LOG_INFO("BPM Kanäle: " + std::to_string(m_numBpmChannels) + 
             ", VU Kanäle: " + std::to_string(m_numVuChannels));
    
    // Feature Toggles
    m_enableBeatclock = env.getInt("ENABLE_BEATCLOCK", 1) != 0;
    m_enableVu = env.getInt("ENABLE_VU", 1) != 0;
    m_enableBeat = env.getInt("ENABLE_BEAT", 1) != 0;
    
    // Debug Flags
    m_debugVuConsole = env.getInt("DEBUG_VU_CONSOLE", 0) != 0;
    m_debugBeatConsole = env.getInt("DEBUG_BEAT_CONSOLE", 1) != 0;
    m_debugBtrackConsole = env.getInt("DEBUG_BTRACK_CONSOLE", 0) != 0;
    m_debugPioneerConsole = env.getInt("DEBUG_PIONEER_CONSOLE", 0) != 0;
    
    // VU-Meter Konfiguration
    m_vuRmsAttack = env.getFloat("VU_RMS_ATTACK", 0.8f);
    m_vuRmsRelease = env.getFloat("VU_RMS_RELEASE", 0.2f);
    m_vuPeakFalloff = env.getFloat("VU_PEAK_FALLOFF", 20.0f);
    
    // OSC Sende-Rate
    m_oscSendRate = env.getInt("OSC_SEND_RATE", 25);
    if (m_oscSendRate < 1) m_oscSendRate = 1;
    if (m_oscSendRate > 100) m_oscSendRate = 100;
    m_oscSendIntervalMs = 1000 / m_oscSendRate;
    LOG_INFO("OSC Sende-Rate: " + std::to_string(m_oscSendRate) + " Hz (" + 
             std::to_string(m_oscSendIntervalMs) + "ms)");
    
    // BPM-Limits
    m_bpmMin = env.getFloat("BPM_MIN", 60.0f);
    m_bpmMax = env.getFloat("BPM_MAX", 200.0f);
    
    LOG_INFO("Features: Beatclock=" + std::string(m_enableBeatclock ? "ON" : "OFF") +
             " VU=" + std::string(m_enableVu ? "ON" : "OFF") +
             " Beat=" + std::string(m_enableBeat ? "ON" : "OFF") +
             " BPM=" + std::to_string(static_cast<int>(m_bpmMin)) + 
             "-" + std::to_string(static_cast<int>(m_bpmMax)));
}

// ============================================================================
// Subsystem-Initialisierung
// ============================================================================

void BeatAnalyzerApp::initVuMeters() {
    for (int i = 0; i < m_numVuChannels; ++i) {
        auto vuMeter = std::make_unique<VuMeter>();
        vuMeter->setRmsAttack(m_vuRmsAttack);
        vuMeter->setRmsRelease(m_vuRmsRelease);
        vuMeter->setPeakFalloff(m_vuPeakFalloff);
        m_vuMeters.push_back(std::move(vuMeter));
        m_vuTrackStates.push_back(VuTrackState{});
        m_vuOscPaths.push_back("/vu/" + std::to_string(i));
    }
}

void BeatAnalyzerApp::initBeatTrackers(int hopSize, int frameSize) {
    for (int i = 0; i < m_numBpmChannels; ++i) {
        auto btrack = std::make_unique<::BTrackWrapper>(hopSize, frameSize);
        m_btrackDetectors.push_back(std::move(btrack));
        m_bpmTrackStates.push_back(BpmTrackState{});
    }
    
    LOG_INFO(std::to_string(m_numBpmChannels) + " Beat Tracker, " + 
             std::to_string(m_numVuChannels) + " VU-Meter initialisiert");
}

void BeatAnalyzerApp::initOscSender(EnvConfig& env) {
    m_oscSender = std::make_shared<OscSender>();
    
    auto oscHostKeys = env.getKeysWithPrefix("OSC_HOST_");
    if (!oscHostKeys.empty()) {
        for (const auto& key : oscHostKeys) {
            std::string value = env.getString(key, "");
            if (value.empty()) continue;
            
            std::string host;
            int port;
            if (parseHostPort(value, host, port)) {
                std::string name = key.substr(9);  // Nach "OSC_HOST_"
                m_oscSender->addTarget(name, host, port);
            }
        }
    } else {
        std::string oscHost = env.getString("OSC_HOST", "127.0.0.1");
        int oscPort = env.getInt("OSC_PORT", 9000);
        m_oscSender->addTarget("default", oscHost, oscPort);
    }
    
    if (!m_oscSender->initialize()) {
        LOG_WARN("OSC Sender konnte nicht initialisiert werden - OSC deaktiviert");
    } else {
        LOG_INFO("OSC aktiv mit " + std::to_string(m_oscSender->getTargetCount()) + " Ziel(en)");
    }
}

void BeatAnalyzerApp::initOscReceiver(EnvConfig& env) {
    int portA3motion = env.getInt("OSC_PORT_A3MOTION", 7775);
    
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
                m_oscSender->sendBeatClockExcept(msg, "motion");
            }
        }
    });
    
    // /clockmode i
    m_oscReceiverA3motion->setClockModeCallback([this](int mode) {
        if (mode < 0 || mode > 2) mode = 1;
        m_clockMode.store(mode);
        const char* modeNames[] = {"a3motion", "intern", "pioneer"};
        LOG_INFO("Clock-Modus: " + std::to_string(mode) + " (" + modeNames[mode] + ")");
    });
    
    // /tap i
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
}

void BeatAnalyzerApp::initPioneerReceiver(EnvConfig& env) {
    int pioneerDeviceNum = env.getInt("PIONEER_DEVICE_NUM", 7);
    m_pioneerReceiver = std::make_unique<PioneerReceiver>();
    m_pioneerReceiver->setDeviceNumber(static_cast<uint8_t>(pioneerDeviceNum));
    m_pioneerReceiver->setDeviceName("beat-analyzer");
    
    // Beat von Pioneer: nur in Modus 2 weiterleiten
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
                msg.bar_number = 0;
                msg.bpm = beat.effectiveBpm;
                m_oscSender->sendBeatClock(msg);
            }
            
            if (m_debugPioneerConsole) {
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
}

} // namespace BeatAnalyzer
