#pragma once

/**
 * BeatAnalyzerApp — Zentrale Anwendungsklasse
 *
 * Orchestriert alle Subsysteme:
 *   JACK Audio → Ringbuffer → BTrack Beat Detection → Synthclock → OSC Output
 *   Pioneer DJ Link → OSC Output
 *   a3motion OSC → OSC Output
 *   VU-Meter → OSC Output
 */

#include "audio/jack_client.h"
#include "analysis/btrack_wrapper.h"
#include "analysis/vu_meter.h"
#include "osc/osc_sender.h"
#include "osc/osc_receiver.h"
#include "osc/pioneer_receiver.h"
#include "config/env_config.h"
#include "util/logging.h"

#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <vector>
#include <cmath>
#include <cstring>
#include <array>
#include <algorithm>

namespace BeatAnalyzer {

// Globaler Shutdown-Flag (gesetzt von Signal-Handler)
extern std::atomic<bool> g_running;

class BeatAnalyzerApp {
public:
    BeatAnalyzerApp();
    
    bool initialize();
    bool run();
    void shutdown();
    
private:
    // ================================================================
    // Structs
    // ================================================================
    
    // Track-Status für BPM Kanäle
    struct BpmTrackState {
        // SYNTHBEAT Clock-State
        double lastBeatFrame = 0.0;
        double synthPhase = 0.0;             // Phase-Akkumulator (0.0-1.0)
        int beatNumber = 1;                  // 1-4 (Schlag im Takt)
        int barNumber = 1;                   // Takt-Nummer (fortlaufend)
        
        // BPM
        double currentBpm = 120.0;           // Aktive BPM für SYNTHBEAT
        double acfBpm = 0.0;
        
        // GRID State
        double gridBpm = 0.0;
        double gridPhase = 0.0;
        double gridConfidence = 0.0;
        int64_t lastGridUpdateFrame = 0;
        
        // SOLL-GRID State
        double sollGridBpm = 0.0;
        double sollGridPhase = 0.0;
        double sollGridConfidence = 0.0;
        int64_t sollGridFrame = 0;
        
        // TAP Pattern
        double tapPatternBpm = 0.0;
        int64_t tapPatternValidUntil = 0;
        double tapLockedBpm = 0.0;
        bool autoMaskCalculated = false;
        
        // REALBEAT Tracking
        int64_t lastRealBeatFrame = 0;
        
        // Legacy
        double recentBpmValues[8] = {0};
        int bpmValueIndex = 0;
        int bpmValueCount = 0;
        int consecutiveRegularBeats = 0;
        
        // Signal
        bool hasSignal = false;
        int64_t lastSignalFrame = 0;
        bool wasInPause = true;
    };
    
    struct VuTrackState {
        bool hasSignal = false;
    };
    
    struct TapEvent {
        int beat;
        std::chrono::steady_clock::time_point timestamp;
    };
    
    // Lock-free Audio Ringbuffer: JACK-Callback → BTrack-Thread
    static constexpr int AUDIO_RING_SIZE = 64;
    static constexpr int AUDIO_RING_MASK = AUDIO_RING_SIZE - 1;
    static constexpr int MAX_FRAME_SIZE = 1024;
    
    struct AudioSlot {
        float data[MAX_FRAME_SIZE];
        int frameCount = 0;
    };
    
    struct AudioRing {
        AudioSlot slots[AUDIO_RING_SIZE];
        alignas(64) std::atomic<int> wpos{0};
        alignas(64) std::atomic<int> rpos{0};
    };
    
    // ================================================================
    // Methoden (implementiert in beat_analyzer_app.cpp)
    // ================================================================
    
    void loadConfig(EnvConfig& env);
    void initVuMeters();
    void initBeatTrackers(int hopSize, int frameSize);
    void initOscSender(EnvConfig& env);
    void initOscReceiver(EnvConfig& env);
    void initPioneerReceiver(EnvConfig& env);
    
    // ================================================================
    // Methoden (implementiert in beat_processing.cpp)
    // ================================================================
    
    /// JACK Audio Callback — nur Ringbuffer-Copy, kein Processing
    void processAudio(const std::vector<const Audio::CSAMPLE*>& bpmBuffers,
                      const std::vector<const Audio::CSAMPLE*>& vuBuffers,
                      int frameCount);
    
    /// Beat-Thread: BTrack + TAP + Synthclock + OSC
    void processBeatThread();
    
    /// Beat-Clock per OSC senden (Modus 1: intern)
    void sendBeatClockForChannel(int ch, bool isRealBeat = false);
    
    /// VU-Meter per OSC senden
    void sendVuMeterOsc();
    
    // ================================================================
    // Konstanten
    // ================================================================
    
    static constexpr int MAX_BPM_CHANNELS = 8;
    static constexpr int MAX_VU_CHANNELS = 16;
    static constexpr int TAP_QUEUE_SIZE = 16;
    static constexpr double BEAT_TOLERANCE = 0.20;
    static constexpr int PHASE_LOCK_BEATS = 3;
    
    // ================================================================
    // Member-Variablen
    // ================================================================
    
    // Audio
    std::shared_ptr<Audio::JackClient> m_jackClient;
    int m_numBpmChannels;
    int m_numVuChannels;
    
    // Beat-Kommunikation JACK-Callback → Beat-Thread (lock-free)
    std::atomic<bool> m_btrackBeatFlag[MAX_BPM_CHANNELS] = {};
    std::atomic<double> m_btrackBpmValue[MAX_BPM_CHANNELS] = {};
    int64_t m_lastProcessedFrame[MAX_BPM_CHANNELS] = {};
    
    // Audio Ringbuffer (SPSC pro Kanal)
    AudioRing m_audioRing[MAX_BPM_CHANNELS];
    
    // Beat Detection
    std::vector<std::unique_ptr<::BTrackWrapper>> m_btrackDetectors;
    std::vector<BpmTrackState> m_bpmTrackStates;
    
    // VU-Meter
    std::vector<std::unique_ptr<VuMeter>> m_vuMeters;
    std::vector<VuTrackState> m_vuTrackStates;
    std::vector<std::string> m_vuOscPaths;
    
    // OSC
    std::shared_ptr<OSC::OscSender> m_oscSender;
    std::unique_ptr<OSC::OscReceiver> m_oscReceiverA3motion;
    std::unique_ptr<OSC::PioneerReceiver> m_pioneerReceiver;
    
    // Clock-Modus: 0=a3motion, 1=intern, 2=pioneer
    std::atomic<int> m_clockMode{1};
    
    // Tap-Queue (SPSC Ringbuffer)
    TapEvent m_tapQueue[TAP_QUEUE_SIZE];
    std::atomic<int> m_tapQueueWrite{0};
    std::atomic<int> m_tapQueueRead{0};
    
    // Tap-Timing (nur im Beat-Thread)
    std::chrono::steady_clock::time_point m_lastTapTime{};
    bool m_hasPrevTap = false;
    double m_tapBpm = 0.0;
    double m_tapIntervals[8] = {0};
    int m_tapIntervalIndex = 0;
    int m_tapIntervalCount = 0;
    
    // Status
    std::mutex m_mutex;
    std::atomic<int64_t> m_frameCount{0};
    
    // Feature Toggles
    bool m_enableBeatclock = true;
    bool m_enableVu = true;
    bool m_enableBeat = true;
    
    // Debug Flags
    bool m_debugVuConsole = false;
    bool m_debugBeatConsole = true;
    bool m_debugBtrackConsole = false;
    bool m_debugPioneerConsole = false;
    
    // VU-Meter Konfiguration
    float m_vuRmsAttack = 0.8f;
    float m_vuRmsRelease = 0.2f;
    float m_vuPeakFalloff = 20.0f;
    
    // OSC Sende-Rate
    int m_oscSendRate = 25;
    int m_oscSendIntervalMs = 40;
    
    // BPM-Limits
    float m_bpmMin = 60.0f;
    float m_bpmMax = 200.0f;
};

} // namespace BeatAnalyzer
