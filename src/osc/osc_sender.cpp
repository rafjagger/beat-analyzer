#include "osc/osc_sender.h"
#include "util/logging.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>

namespace BeatAnalyzer {
namespace OSC {

// ============================================================================
// OSC binary serialization (minimal, just what we need)
// ============================================================================

int OscSender::padLen(int len) {
    return (len + 3) & ~3;  // round up to next multiple of 4
}

int OscSender::writeOscString(char* buf, const char* str) {
    int slen = static_cast<int>(std::strlen(str));
    int padded = padLen(slen + 1);  // +1 for null terminator
    std::memcpy(buf, str, slen);
    std::memset(buf + slen, 0, padded - slen);  // null-pad
    return padded;
}

// Write big-endian int32
static void writeInt32(char* buf, int32_t val) {
    uint32_t v = static_cast<uint32_t>(val);
    buf[0] = static_cast<char>((v >> 24) & 0xFF);
    buf[1] = static_cast<char>((v >> 16) & 0xFF);
    buf[2] = static_cast<char>((v >>  8) & 0xFF);
    buf[3] = static_cast<char>((v      ) & 0xFF);
}

// Write big-endian float32
static void writeFloat32(char* buf, float val) {
    uint32_t v;
    std::memcpy(&v, &val, 4);
    writeInt32(buf, static_cast<int32_t>(v));
}

int OscSender::serializeInts(char* buf, const char* path, int i1, int i2, int i3) {
    int pos = 0;
    pos += writeOscString(buf + pos, path);
    pos += writeOscString(buf + pos, ",iii");
    writeInt32(buf + pos, i1); pos += 4;
    writeInt32(buf + pos, i2); pos += 4;
    writeInt32(buf + pos, i3); pos += 4;
    return pos;
}

int OscSender::serializeFloat(char* buf, const char* path, float f1) {
    int pos = 0;
    pos += writeOscString(buf + pos, path);
    pos += writeOscString(buf + pos, ",f");
    writeFloat32(buf + pos, f1); pos += 4;
    return pos;
}

int OscSender::serializeFloats(char* buf, const char* path, float f1, float f2) {
    int pos = 0;
    pos += writeOscString(buf + pos, path);
    pos += writeOscString(buf + pos, ",ff");
    writeFloat32(buf + pos, f1); pos += 4;
    writeFloat32(buf + pos, f2); pos += 4;
    return pos;
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

OscSender::OscSender() = default;

OscSender::~OscSender() {
    shutdown();
}

// ============================================================================
// Setup
// ============================================================================

void OscSender::addTarget(const std::string& name, const std::string& host, int port) {
    auto t = std::make_unique<Target>();
    t->name = name;
    t->host = host;
    t->port = port;
    m_targets.push_back(std::move(t));
}

bool OscSender::initialize() {
    if (m_targets.empty()) {
        LOG_WARN("No OSC targets configured");
        return false;
    }
    
    bool anyOk = false;
    
    for (auto& t : m_targets) {
        // Resolve hostname to IP once
        std::string ip = t->host;
        struct addrinfo hints{}, *res = nullptr;
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        
        if (getaddrinfo(t->host.c_str(), nullptr, &hints, &res) == 0 && res) {
            char ipStr[INET_ADDRSTRLEN];
            auto* sa = reinterpret_cast<struct sockaddr_in*>(res->ai_addr);
            inet_ntop(AF_INET, &sa->sin_addr, ipStr, sizeof(ipStr));
            ip = ipStr;
            freeaddrinfo(res);
        }
        
        // Create non-blocking UDP socket
        t->sockfd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
        if (t->sockfd < 0) {
            LOG_ERROR("OSC: Socket-Fehler für " + t->name);
            continue;
        }
        
        // Increase send buffer to reduce drops
        int sndbuf = 262144;  // 256KB
        setsockopt(t->sockfd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
        
        // Target address
        std::memset(&t->addr, 0, sizeof(t->addr));
        t->addr.sin_family = AF_INET;
        t->addr.sin_port = htons(static_cast<uint16_t>(t->port));
        inet_pton(AF_INET, ip.c_str(), &t->addr.sin_addr);
        
        // Start sender thread
        t->running = true;
        t->thread = std::thread(targetThreadFunc, t.get());
        
        LOG_INFO("OSC target (UDP): " + t->name + " -> " + ip + ":" + std::to_string(t->port));
        anyOk = true;
    }
    
    m_connected = anyOk;
    return anyOk;
}

void OscSender::shutdown() {
    for (auto& t : m_targets) {
        t->running = false;
    }
    for (auto& t : m_targets) {
        if (t->thread.joinable()) t->thread.join();
        if (t->sockfd >= 0) {
            close(t->sockfd);
            t->sockfd = -1;
        }
        uint32_t d = t->dropped.load();
        if (d > 0) {
            LOG_WARN("OSC " + t->name + ": " + std::to_string(d) + " packets dropped");
        }
    }
}

// ============================================================================
// Lock-free enqueue to all targets
// ============================================================================

void OscSender::enqueueAll(const char* data, int len) {
    for (auto& t : m_targets) {
        int w = t->wpos.load(std::memory_order_relaxed);
        int next = (w + 1) & QMASK;
        
        // If full, overwrite oldest (reader will skip ahead)
        // Better: just drop this packet for this target
        if (next == t->rpos.load(std::memory_order_acquire)) {
            t->dropped.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        
        auto& pkt = t->ring[w];
        std::memcpy(pkt.data, data, len);
        pkt.len = len;
        
        t->wpos.store(next, std::memory_order_release);
    }
}

// ============================================================================
// Public send methods — serialize + enqueue
// ============================================================================

bool OscSender::sendBeatClock(const BeatClockMessage& msg) {
    if (!m_connected) return false;
    char buf[256];
    int len = serializeInts(buf, "/beat", msg.beat_number, msg.bar_number, msg.bpm);
    enqueueAll(buf, len);
    return true;
}

bool OscSender::sendFloat(const std::string& path, float value) {
    if (!m_connected) return false;
    char buf[256];
    int len = serializeFloat(buf, path.c_str(), value);
    enqueueAll(buf, len);
    return true;
}

bool OscSender::sendFloats(const std::string& path, float value1, float value2) {
    if (!m_connected) return false;
    char buf[256];
    int len = serializeFloats(buf, path.c_str(), value1, value2);
    enqueueAll(buf, len);
    return true;
}

bool OscSender::sendMessage(const OscMessage& msg) {
    if (!m_connected) return false;
    
    // Serialize generic message
    char buf[256];
    int pos = 0;
    pos += writeOscString(buf + pos, msg.path.c_str());
    
    // Build type tag string
    std::string types = ",";
    for (const auto& arg : msg.args) {
        try {
            std::stoi(arg);
            types += 'i';
        } catch (...) {
            try {
                std::stof(arg);
                types += 'f';
            } catch (...) {
                types += 's';
            }
        }
    }
    pos += writeOscString(buf + pos, types.c_str());
    
    // Write arguments
    for (const auto& arg : msg.args) {
        if (pos >= 240) break;  // safety margin
        try {
            int v = std::stoi(arg);
            writeInt32(buf + pos, v); pos += 4;
        } catch (...) {
            try {
                float v = std::stof(arg);
                writeFloat32(buf + pos, v); pos += 4;
            } catch (...) {
                pos += writeOscString(buf + pos, arg.c_str());
            }
        }
    }
    
    enqueueAll(buf, pos);
    return true;
}

bool OscSender::broadcastBeatClock(int64_t, float bpm, int beatNumber, float) {
    BeatClockMessage msg;
    msg.track_id = 0;
    msg.beat_number = beatNumber;
    msg.bar_number = 1;
    msg.bpm = static_cast<int>(bpm + 0.5f);
    return sendBeatClock(msg);
}

// ============================================================================
// Target thread: drain ringbuffer, sendto() on non-blocking socket
// ============================================================================

void OscSender::targetThreadFunc(Target* t) {
    while (t->running) {
        int r = t->rpos.load(std::memory_order_relaxed);
        int w = t->wpos.load(std::memory_order_acquire);
        
        if (r == w) {
            // Nothing to send — sleep briefly
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            continue;
        }
        
        // Send all queued packets
        while (r != w) {
            auto& pkt = t->ring[r];
            // Fire and forget — ignore errors
            sendto(t->sockfd, pkt.data, pkt.len, MSG_DONTWAIT,
                   reinterpret_cast<const struct sockaddr*>(&t->addr),
                   sizeof(t->addr));
            r = (r + 1) & QMASK;
        }
        
        t->rpos.store(r, std::memory_order_release);
    }
}

} // namespace OSC
} // namespace BeatAnalyzer
