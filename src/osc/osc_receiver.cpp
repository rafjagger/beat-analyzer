#include "osc/osc_receiver.h"
#include "util/logging.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <chrono>
#include <algorithm>

namespace BeatAnalyzer {
namespace OSC {

// ============================================================================
// OSC binary reading helpers
// ============================================================================

int OscReceiver::readOscString(const char* buf, int len, const char*& out) {
    // OSC string: null-terminated, padded to 4 bytes
    out = buf;
    int slen = static_cast<int>(strnlen(buf, len));
    if (slen >= len) return -1;
    int padded = (slen + 4) & ~3;  // round up to next 4 bytes
    return (padded <= len) ? padded : -1;
}

int32_t OscReceiver::readInt32(const char* buf) {
    uint32_t v = (static_cast<uint8_t>(buf[0]) << 24) |
                 (static_cast<uint8_t>(buf[1]) << 16) |
                 (static_cast<uint8_t>(buf[2]) <<  8) |
                 (static_cast<uint8_t>(buf[3]));
    return static_cast<int32_t>(v);
}

float OscReceiver::readFloat32(const char* buf) {
    int32_t i = readInt32(buf);
    float f;
    std::memcpy(&f, &i, 4);
    return f;
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

OscReceiver::OscReceiver() = default;

OscReceiver::~OscReceiver() {
    stop();
}

void OscReceiver::setPort(int port) { m_port = port; }
void OscReceiver::setBeatClockPath(const std::string& path) { m_beatClockPath = path; }
void OscReceiver::setCallback(BeatClockCallback callback) { m_callback = callback; }

ReceivedBeatClock OscReceiver::getLastBeatClock() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_lastBeatClock;
}

// ============================================================================
// Start / Stop
// ============================================================================

bool OscReceiver::start() {
    if (m_running) return true;
    
    // Create UDP socket
    m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_sockfd < 0) {
        LOG_ERROR("OSC Receiver: Socket-Fehler");
        return false;
    }
    
    // Allow reuse
    int optval = 1;
    setsockopt(m_sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    
    // Bind
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(m_port));
    addr.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(m_sockfd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        LOG_ERROR("OSC Receiver: Bind fehlgeschlagen auf Port " + std::to_string(m_port));
        close(m_sockfd);
        m_sockfd = -1;
        return false;
    }
    
    m_running = true;
    m_thread = std::thread(&OscReceiver::recvLoop, this);
    
    LOG_INFO("OSC Receiver gestartet auf Port " + std::to_string(m_port) + 
             " (UDP, listening for " + m_beatClockPath + ", /clockmode, /tap)");
    return true;
}

void OscReceiver::stop() {
    if (!m_running) return;
    m_running = false;
    
    // Unblock recvfrom by closing socket
    if (m_sockfd >= 0) {
        ::shutdown(m_sockfd, SHUT_RDWR);
        close(m_sockfd);
        m_sockfd = -1;
    }
    
    if (m_thread.joinable()) {
        m_thread.join();
    }
    
    LOG_INFO("OSC Receiver gestoppt");
}

// ============================================================================
// Receive loop
// ============================================================================

void OscReceiver::recvLoop() {
    char buf[1024];
    
    while (m_running) {
        // poll() with timeout so we can check m_running
        struct pollfd pfd;
        pfd.fd = m_sockfd;
        pfd.events = POLLIN;
        
        int ret = poll(&pfd, 1, 50);  // 50ms timeout
        if (ret <= 0) continue;
        if (!(pfd.revents & POLLIN)) continue;
        
        struct sockaddr_in sender{};
        socklen_t slen = sizeof(sender);
        ssize_t n = recvfrom(m_sockfd, buf, sizeof(buf), 0,
                             reinterpret_cast<struct sockaddr*>(&sender), &slen);
        
        if (n > 0) {
            handlePacket(buf, static_cast<int>(n));
        }
    }
}

// ============================================================================
// OSC packet handler
// ============================================================================

void OscReceiver::handlePacket(const char* data, int len) {
    if (len < 4) return;
    
    // Read OSC path
    const char* path = nullptr;
    int pos = readOscString(data, len, path);
    if (pos < 0 || !path) return;
    
    // Read type tag string (starts with ',')
    const char* types = nullptr;
    int typesLen = 0;
    if (pos < len) {
        typesLen = readOscString(data + pos, len - pos, types);
        if (typesLen > 0) {
            pos += typesLen;
        }
    }
    
    // Skip the ',' in type tag
    const char* typeChars = (types && types[0] == ',') ? types + 1 : "";
    int argc = static_cast<int>(strlen(typeChars));
    int argPos = pos;
    
    // ── /beat iii ──────────────────────────────────────────────
    if (std::strcmp(path, m_beatClockPath.c_str()) == 0) {
        ReceivedBeatClock clock;
        clock.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        clock.valid = true;
        
        for (int i = 0; i < argc && argPos + 4 <= len; i++) {
            if (typeChars[i] == 'i') {
                int32_t val = readInt32(data + argPos);
                if (i == 0) clock.beatNumber = val;
                else if (i == 1) clock.bar = val;
                else if (i == 2) clock.bpm = static_cast<double>(val);
                argPos += 4;
            } else if (typeChars[i] == 'f') {
                float val = readFloat32(data + argPos);
                if (i == 0) clock.beatNumber = static_cast<int>(val);
                else if (i == 1) clock.bar = static_cast<int>(val);
                else if (i == 2) clock.bpm = static_cast<double>(val);
                argPos += 4;
            }
        }
        
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_lastBeatClock = clock;
        }
        
        if (m_callback) {
            m_callback(clock);
        }
    }
    // ── /clockmode i|f ────────────────────────────────────────
    else if (std::strcmp(path, "/clockmode") == 0) {
        int mode = 0;
        if (argc >= 1 && argPos + 4 <= len) {
            if (typeChars[0] == 'i') {
                mode = readInt32(data + argPos);
            } else if (typeChars[0] == 'f') {
                mode = static_cast<int>(readFloat32(data + argPos));
            }
        }
        
        mode = std::max(0, std::min(mode, 2));  // Clamp 0-2
        int oldMode = m_clockMode.exchange(mode);
        
        if (oldMode != mode) {
            const char* names[] = {"a3motion", "intern", "pioneer"};
            LOG_INFO("Clock-Modus gewechselt: " + std::string(names[mode]));
        }
        
        if (m_clockModeCallback) {
            m_clockModeCallback(mode);
        }
    }
    // ── /tap [i] ─────────────────────────────────────────────
    else if (std::strcmp(path, "/tap") == 0) {
        int beat = 1;
        if (argc >= 1 && argPos + 4 <= len) {
            if (typeChars[0] == 'i') {
                beat = readInt32(data + argPos);
            } else if (typeChars[0] == 'f') {
                beat = static_cast<int>(readFloat32(data + argPos));
            }
        }
        
        if (beat < 1 || beat > 4) beat = 1;
        
        LOG_INFO("TAP empfangen -> Beat " + std::to_string(beat));
        
        if (m_tapCallback) {
            m_tapCallback(beat);
        }
    }
}

} // namespace OSC
} // namespace BeatAnalyzer
