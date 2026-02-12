#pragma once

#include <string>
#include <functional>
#include <atomic>
#include <mutex>
#include <thread>
#include <cstdint>

namespace BeatAnalyzer {
namespace OSC {

/**
 * Empfangene Beat-Clock Daten
 */
struct ReceivedBeatClock {
    int beatNumber = 0;     // 1-4
    int bar = 1;            // Takt
    double bpm = 0.0;       // BPM (z.B. 128.53)
    int64_t timestamp = 0;  // ms since epoch
    bool valid = false;
};

/**
 * OSC Receiver — raw UDP socket, kein liblo.
 * 
 * Eigener Thread mit recvfrom() + minimalem OSC-Parser.
 * Zwei Instanzen möglich (verschiedene Ports):
 *   - a3motion (Port 7775): /beat, /clockmode, /tap
 *   - Pioneer  (Port 7776): /beat
 * 
 * Empfängt:
 *   /beat iii       beat, bar, bpm
 *   /clockmode i    0=a3motion, 1=intern, 2=pioneer
 *   /tap [i]        Setzt Beat (Default: 1)
 */
class OscReceiver {
public:
    using BeatClockCallback = std::function<void(const ReceivedBeatClock&)>;
    using ClockModeCallback = std::function<void(int mode)>;
    using TapCallback = std::function<void(int beat)>;
    
    OscReceiver();
    ~OscReceiver();
    
    OscReceiver(const OscReceiver&) = delete;
    OscReceiver& operator=(const OscReceiver&) = delete;
    
    void setPort(int port);
    void setBeatClockPath(const std::string& path);
    void setCallback(BeatClockCallback callback);
    void setClockModeCallback(ClockModeCallback callback) { m_clockModeCallback = callback; }
    void setTapCallback(TapCallback callback) { m_tapCallback = callback; }
    
    bool start();
    void stop();
    
    bool isRunning() const { return m_running; }
    int getPort() const { return m_port; }
    
    ReceivedBeatClock getLastBeatClock() const;
    
    int getClockMode() const { return m_clockMode.load(); }
    void setClockMode(int mode) { m_clockMode.store(mode); }
    
private:
    int m_port = 7775;
    std::string m_beatClockPath = "/beat";
    int m_sockfd = -1;
    std::thread m_thread;
    std::atomic<bool> m_running{false};
    std::atomic<int> m_clockMode{0};
    
    mutable std::mutex m_mutex;
    ReceivedBeatClock m_lastBeatClock;
    
    BeatClockCallback m_callback;
    ClockModeCallback m_clockModeCallback;
    TapCallback m_tapCallback;
    
    void recvLoop();
    void handlePacket(const char* data, int len);
    
    // Minimal OSC parsing helpers
    static int readOscString(const char* buf, int len, const char*& out);
    static int32_t readInt32(const char* buf);
    static float readFloat32(const char* buf);
};

} // namespace OSC
} // namespace BeatAnalyzer
