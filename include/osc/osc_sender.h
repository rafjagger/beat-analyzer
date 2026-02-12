#pragma once

#include <string>
#include <memory>
#include <functional>
#include <vector>
#include <atomic>
#include <thread>
#include <cstring>
#include <netinet/in.h>
#include "osc_messages.h"

namespace BeatAnalyzer {
namespace OSC {

/**
 * OSC sender: one thread + socket per target host.
 * 
 * - send*() serializes OSC binary, copies into each target's ringbuffer.
 * - Each target thread does sendto() on its own non-blocking UDP socket.
 * - If a host is unreachable, only its thread is affected.
 * - No liblo for sending — raw UDP sendto(). Fast, simple, no surprises.
 */
class OscSender {
public:
    OscSender();
    ~OscSender();
    
    OscSender(const OscSender&) = delete;
    OscSender& operator=(const OscSender&) = delete;
    
    void addTarget(const std::string& name, const std::string& host, int port);
    bool initialize();
    void shutdown();
    
    bool sendBeatClock(const BeatClockMessage& msg);
    /** Send /beat to all targets EXCEPT the named one */
    bool sendBeatClockExcept(const BeatClockMessage& msg, const std::string& excludeTarget);
    bool sendFloat(const std::string& path, float value);
    bool sendFloats(const std::string& path, float value1, float value2);
    bool sendMessage(const OscMessage& msg);
    bool broadcastBeatClock(int64_t framePos, float bpm, int beatNumber, float strength);
    
    /** Send all VU channels as a single OSC bundle.
     *  One UDP packet instead of N. Atomic delivery, less overhead.
     *  paths[i] = "/vu/0" etc, peaks[i]/rms[i] = values per channel. */
    bool sendVuBundle(const std::string* paths, const float* peaks, const float* rms, int numChannels);
    
    bool isConnected() const { return m_connected; }
    size_t getTargetCount() const { return m_targets.size(); }
    
    using ErrorCallback = std::function<void(const std::string&)>;
    void setErrorCallback(ErrorCallback cb) { m_errorCallback = cb; }
    
private:
    struct Packet {
        char data[512];   // 512 for VU bundle (12ch ≈ 304 bytes)
        int len = 0;
    };
    
    static constexpr int QSIZE = 1024;  // power of 2
    static constexpr int QMASK = QSIZE - 1;
    
    struct Target {
        std::string name;
        std::string host;
        int port = 0;
        int sockfd = -1;
        struct sockaddr_in addr;
        
        // SPSC ringbuffer
        Packet ring[QSIZE];
        alignas(64) std::atomic<int> wpos{0};
        alignas(64) std::atomic<int> rpos{0};
        
        std::thread thread;
        std::atomic<bool> running{false};
        std::atomic<uint32_t> dropped{0};
    };
    
    std::vector<std::unique_ptr<Target>> m_targets;
    bool m_connected = false;
    ErrorCallback m_errorCallback;
    
    // OSC binary serialization helpers
    static int padLen(int len);  // round up to multiple of 4
    static int writeOscString(char* buf, const char* str);
    static int serializeInts(char* buf, const char* path, int i1, int i2, int i3);
    static int serializeIntIntFloat(char* buf, const char* path, int i1, int i2, float f1);
    static int serializeFloat(char* buf, const char* path, float f1);
    static int serializeFloats(char* buf, const char* path, float f1, float f2);
    static int serializeBundle(char* buf, int bufSize, const std::string* paths, const float* peaks, const float* rms, int numChannels);
    
    void enqueueAll(const char* data, int len);
    void enqueueAllExcept(const char* data, int len, const std::string& excludeName);
    static void targetThreadFunc(Target* t);
};

using OscSenderPtr = std::shared_ptr<OscSender>;

} // namespace OSC
} // namespace BeatAnalyzer
