#pragma once

#include <string>
#include <memory>
#include <functional>
#include <vector>
#include <atomic>
#include <thread>
#include <array>
#include <mutex>
#include <condition_variable>
#include "osc_messages.h"

namespace BeatAnalyzer {
namespace OSC {

/**
 * Single OSC target address.
 */
struct OscTarget {
    std::string name;
    std::string host;
    int port;
    void* address = nullptr;  // lo_address from liblo
};

/**
 * OSC sender supporting multiple targets.
 * 
 * THREAD-SAFE: Alle send*()-Methoden schreiben in eine lock-free Queue.
 * Ein dedizierter Sender-Thread liest die Queue und macht alle lo_send-Aufrufe.
 * Dadurch kein Blocking in VU/Beat/OSC-Threads.
 */
class OscSender {
public:
    OscSender();
    ~OscSender();
    
    // Non-copyable
    OscSender(const OscSender&) = delete;
    OscSender& operator=(const OscSender&) = delete;
    
    // Add a target host
    void addTarget(const std::string& name, const std::string& host, int port);
    
    // Initialize all OSC targets and start sender thread
    bool initialize();
    
    // Stop sender thread
    void shutdown();
    
    // Send beat clock message (thread-safe, non-blocking)
    bool sendBeatClock(const BeatClockMessage& msg);
    
    // Send single float value to path (thread-safe, non-blocking)
    bool sendFloat(const std::string& path, float value);
    
    // Send two float values to path (thread-safe, non-blocking)
    bool sendFloats(const std::string& path, float value1, float value2);
    
    // Send raw OSC message (thread-safe, non-blocking)
    bool sendMessage(const OscMessage& msg);
    
    // Send to all tracks (0-3) on all targets
    bool broadcastBeatClock(
        int64_t framePos,
        float bpm,
        int beatNumber,
        float strength);
    
    // Connection properties
    bool isConnected() const { return m_connected; }
    size_t getTargetCount() const { return m_targets.size(); }
    const std::vector<OscTarget>& getTargets() const { return m_targets; }
    
    // Set error callback for debugging
    using ErrorCallback = std::function<void(const std::string&)>;
    void setErrorCallback(ErrorCallback callback) {
        m_errorCallback = callback;
    }
    
private:
    std::vector<OscTarget> m_targets;
    bool m_connected;
    ErrorCallback m_errorCallback;
    
    // === Lock-free Send Queue ===
    // Nachrichtentypen für die Queue
    enum class QueueMsgType : uint8_t {
        BEAT_CLOCK,   // path + 3x int (beat, bar, bpm)
        FLOAT,        // path + 1x float
        FLOAT2,       // path + 2x float
        GENERIC       // path + args als strings
    };
    
    struct QueueEntry {
        QueueMsgType type;
        char path[64];          // OSC path (z.B. "/beat", "/vu/0")
        union {
            struct { int i1, i2, i3; } ints;        // BEAT_CLOCK
            struct { float f1; } oneFloat;           // FLOAT
            struct { float f1, f2; } twoFloats;      // FLOAT2
        } data;
        // Für GENERIC: OscMessage wird inline gespeichert (selten genutzt)
        OscMessage genericMsg;
        std::atomic<bool> ready{false};  // true = Eintrag bereit zum Senden
    };
    
    // Ring-Buffer mit fester Größe (keine Allokation beim Senden)
    static constexpr int QUEUE_SIZE = 512;  // Muss Power-of-2 sein
    static constexpr int QUEUE_MASK = QUEUE_SIZE - 1;
    std::array<QueueEntry, QUEUE_SIZE> m_queue;
    std::atomic<int> m_queueWritePos{0};
    std::atomic<int> m_queueReadPos{0};
    
    // Sender-Thread
    std::thread m_senderThread;
    std::atomic<bool> m_senderRunning{false};
    std::mutex m_queueMutex;
    std::condition_variable m_queueCV;
    void senderThreadFunc();
    
    // Interne Send-Funktionen (nur vom Sender-Thread aufgerufen)
    void doSendInts(const char* path, int i1, int i2, int i3);
    void doSendFloat(const char* path, float f1);
    void doSendFloats(const char* path, float f1, float f2);
    void doSendGeneric(const OscMessage& msg);
};

using OscSenderPtr = std::shared_ptr<OscSender>;

} // namespace OSC
} // namespace BeatAnalyzer
