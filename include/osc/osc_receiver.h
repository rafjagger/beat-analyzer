#pragma once

#include <string>
#include <functional>
#include <atomic>
#include <mutex>
#include <cstdint>

#ifdef HAS_LIBLO
#include <lo/lo.h>
#else
// Stub types when liblo not available
typedef void* lo_arg;
typedef void* lo_message;
#endif

namespace BeatAnalyzer {
namespace OSC {

/**
 * Empfangene Beat-Clock Daten
 */
struct ReceivedBeatClock {
    float bpm = 0.0f;
    int beatNumber = 0;
    int64_t timestamp = 0;  // ms since epoch
    bool valid = false;
};

/**
 * OSC Receiver - empfängt externe Beat-Clock und Steuerbefehle
 * 
 * Empfängt:
 *   /beatclock [bpm] [beat_number] - Externe Beat-Clock
 *   /clockmode [0|1]               - 0=Normal, 1=Training
 */
class OscReceiver {
public:
    using BeatClockCallback = std::function<void(const ReceivedBeatClock&)>;
    using ClockModeCallback = std::function<void(int mode)>;
    
    OscReceiver();
    ~OscReceiver();
    
    // Nicht kopierbar
    OscReceiver(const OscReceiver&) = delete;
    OscReceiver& operator=(const OscReceiver&) = delete;
    
    // Konfiguration
    void setPort(int port);
    void setBeatClockPath(const std::string& path);
    void setCallback(BeatClockCallback callback);
    void setClockModeCallback(ClockModeCallback callback) { m_clockModeCallback = callback; }
    
    // Start/Stop
    bool start();
    void stop();
    
    // Status
    bool isRunning() const { return m_running; }
    int getPort() const { return m_port; }
    
    // Letzte empfangene Beat-Clock
    ReceivedBeatClock getLastBeatClock() const;
    
    // Clock-Modus (0=Normal, 1=Training)
    int getClockMode() const { return m_clockMode.load(); }
    void setClockMode(int mode) { m_clockMode.store(mode); }
    
private:
    int m_port = 7775;
    std::string m_beatClockPath = "/beatclock";
    void* m_serverThread = nullptr;
    std::atomic<bool> m_running{false};
    std::atomic<int> m_clockMode{0};
    
    mutable std::mutex m_mutex;
    ReceivedBeatClock m_lastBeatClock;
    
    BeatClockCallback m_callback;
    ClockModeCallback m_clockModeCallback;
    
    static int beatClockHandler(const char* path, const char* types,
                                lo_arg** argv, int argc, lo_message msg, void* userData);
    static int clockModeHandler(const char* path, const char* types,
                                lo_arg** argv, int argc, lo_message msg, void* userData);
};

} // namespace OSC
} // namespace BeatAnalyzer
