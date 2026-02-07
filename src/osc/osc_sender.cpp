#include "osc/osc_sender.h"
#include "util/logging.h"

#ifdef HAS_LIBLO
#include <lo/lo.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <cstring>
#endif

namespace BeatAnalyzer {
namespace OSC {

OscSender::OscSender()
    : m_connected(false) {
    // Queue-Einträge initialisieren
    for (auto& entry : m_queue) {
        entry.ready.store(false, std::memory_order_relaxed);
    }
}

OscSender::~OscSender() {
    shutdown();
    
#ifdef HAS_LIBLO
    for (auto& target : m_targets) {
        if (target.address) {
            lo_address_free(static_cast<lo_address>(target.address));
        }
    }
#endif
}

void OscSender::addTarget(const std::string& name, const std::string& host, int port) {
    OscTarget target;
    target.name = name;
    target.host = host;
    target.port = port;
    target.address = nullptr;
    m_targets.push_back(target);
}

bool OscSender::initialize() {
#ifdef HAS_LIBLO
    if (m_targets.empty()) {
        LOG_WARN("No OSC targets configured");
        return false;
    }
    
    bool anyConnected = false;
    
    for (auto& target : m_targets) {
        // Hostname einmalig zu IP auflösen, damit beim Senden kein DNS-Lookup blockiert
        std::string resolvedHost = target.host;
        struct addrinfo hints, *res;
        std::memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;       // IPv4
        hints.ai_socktype = SOCK_DGRAM;  // UDP
        
        int err = getaddrinfo(target.host.c_str(), nullptr, &hints, &res);
        if (err == 0 && res) {
            char ipStr[INET_ADDRSTRLEN];
            struct sockaddr_in* addr = reinterpret_cast<struct sockaddr_in*>(res->ai_addr);
            inet_ntop(AF_INET, &addr->sin_addr, ipStr, sizeof(ipStr));
            resolvedHost = ipStr;
            freeaddrinfo(res);
            if (resolvedHost != target.host) {
                LOG_INFO("OSC: " + target.host + " aufgelöst zu " + resolvedHost);
            }
        } else {
            LOG_WARN("OSC: DNS-Auflösung fehlgeschlagen für " + target.host + 
                     ", verwende direkt (kann beim Senden blockieren!)");
        }
        
        // Explizit UDP erzwingen - NIEMALS TCP!
        target.address = lo_address_new_with_proto(LO_UDP, 
                                                    resolvedHost.c_str(), 
                                                    std::to_string(target.port).c_str());
        
        if (!target.address) {
            LOG_ERROR("Failed to create OSC address for " + target.name + ": " + 
                      resolvedHost + ":" + std::to_string(target.port));
        } else {
            LOG_INFO("OSC target added (UDP): " + target.name + " -> " + 
                     resolvedHost + ":" + std::to_string(target.port));
            anyConnected = true;
        }
    }
    
    m_connected = anyConnected;
    
    // Sender-Thread starten
    if (m_connected) {
        m_senderRunning = true;
        m_senderThread = std::thread(&OscSender::senderThreadFunc, this);
        LOG_INFO("OSC Sender-Thread gestartet (non-blocking queue)");
    }
    
    return anyConnected;
#else
    LOG_WARN("liblo not available - OSC support disabled");
    m_connected = false;
    return false;
#endif
}

void OscSender::shutdown() {
    if (m_senderRunning) {
        m_senderRunning = false;
        m_queueCV.notify_one();  // Sender-Thread aufwecken damit er beendet
        if (m_senderThread.joinable()) {
            m_senderThread.join();
        }
    }
}

// ============================================================================
// Thread-safe Queue-basierte Send-Methoden (non-blocking)
// ============================================================================

bool OscSender::sendBeatClock(const BeatClockMessage& msg) {
    if (!m_connected) return false;
    
    int writePos = m_queueWritePos.load(std::memory_order_relaxed);
    int nextPos = (writePos + 1) & QUEUE_MASK;
    
    // Queue voll? Drop statt blockieren
    if (nextPos == m_queueReadPos.load(std::memory_order_acquire)) {
        return false;  // Silently drop - besser als blockieren
    }
    
    auto& entry = m_queue[writePos];
    entry.type = QueueMsgType::BEAT_CLOCK;
    std::strncpy(entry.path, "/beat", sizeof(entry.path) - 1);
    entry.path[sizeof(entry.path) - 1] = '\0';
    entry.data.ints.i1 = msg.beat_number;
    entry.data.ints.i2 = msg.bar_number;
    entry.data.ints.i3 = msg.bpm;
    entry.ready.store(true, std::memory_order_release);
    
    m_queueWritePos.store(nextPos, std::memory_order_release);
    m_queueCV.notify_one();
    return true;
}

bool OscSender::sendFloat(const std::string& path, float value) {
    if (!m_connected) return false;
    
    int writePos = m_queueWritePos.load(std::memory_order_relaxed);
    int nextPos = (writePos + 1) & QUEUE_MASK;
    
    if (nextPos == m_queueReadPos.load(std::memory_order_acquire)) {
        return false;
    }
    
    auto& entry = m_queue[writePos];
    entry.type = QueueMsgType::FLOAT;
    std::strncpy(entry.path, path.c_str(), sizeof(entry.path) - 1);
    entry.path[sizeof(entry.path) - 1] = '\0';
    entry.data.oneFloat.f1 = value;
    entry.ready.store(true, std::memory_order_release);
    
    m_queueWritePos.store(nextPos, std::memory_order_release);
    m_queueCV.notify_one();
    return true;
}

bool OscSender::sendFloats(const std::string& path, float value1, float value2) {
    if (!m_connected) return false;
    
    int writePos = m_queueWritePos.load(std::memory_order_relaxed);
    int nextPos = (writePos + 1) & QUEUE_MASK;
    
    if (nextPos == m_queueReadPos.load(std::memory_order_acquire)) {
        return false;
    }
    
    auto& entry = m_queue[writePos];
    entry.type = QueueMsgType::FLOAT2;
    std::strncpy(entry.path, path.c_str(), sizeof(entry.path) - 1);
    entry.path[sizeof(entry.path) - 1] = '\0';
    entry.data.twoFloats.f1 = value1;
    entry.data.twoFloats.f2 = value2;
    entry.ready.store(true, std::memory_order_release);
    
    m_queueWritePos.store(nextPos, std::memory_order_release);
    m_queueCV.notify_one();
    return true;
}

bool OscSender::sendMessage(const OscMessage& msg) {
    if (!m_connected) return false;
    
    int writePos = m_queueWritePos.load(std::memory_order_relaxed);
    int nextPos = (writePos + 1) & QUEUE_MASK;
    
    if (nextPos == m_queueReadPos.load(std::memory_order_acquire)) {
        return false;
    }
    
    auto& entry = m_queue[writePos];
    entry.type = QueueMsgType::GENERIC;
    entry.genericMsg = msg;
    entry.ready.store(true, std::memory_order_release);
    
    m_queueWritePos.store(nextPos, std::memory_order_release);
    m_queueCV.notify_one();
    return true;
}

bool OscSender::broadcastBeatClock(
    int64_t /*framePos*/,
    float bpm,
    int beatNumber,
    float /*strength*/) {
    
    BeatClockMessage msg;
    msg.track_id = 0;
    msg.beat_number = beatNumber;
    msg.bar_number = 1;
    msg.bpm = static_cast<int>(bpm + 0.5f);
    
    return sendBeatClock(msg);
}

// ============================================================================
// Sender-Thread: Einziger Thread der lo_send aufruft
// ============================================================================

void OscSender::senderThreadFunc() {
    while (m_senderRunning) {
        // Warte auf Daten oder Shutdown-Signal
        {
            std::unique_lock<std::mutex> lock(m_queueMutex);
            m_queueCV.wait_for(lock, std::chrono::milliseconds(5), [this]() {
                return !m_senderRunning || 
                       m_queueReadPos.load(std::memory_order_relaxed) != 
                       m_queueWritePos.load(std::memory_order_acquire);
            });
        }
        
        if (!m_senderRunning) break;
        
        int readPos = m_queueReadPos.load(std::memory_order_relaxed);
        int writePos = m_queueWritePos.load(std::memory_order_acquire);
        
        if (readPos == writePos) {
            continue;
        }
        
        // Alle verfügbaren Nachrichten senden
        while (readPos != writePos) {
            auto& entry = m_queue[readPos];
            
            // Warte bis Eintrag wirklich bereit ist
            if (!entry.ready.load(std::memory_order_acquire)) {
                break;
            }
            
            switch (entry.type) {
                case QueueMsgType::BEAT_CLOCK:
                    doSendInts(entry.path, entry.data.ints.i1, entry.data.ints.i2, entry.data.ints.i3);
                    break;
                case QueueMsgType::FLOAT:
                    doSendFloat(entry.path, entry.data.oneFloat.f1);
                    break;
                case QueueMsgType::FLOAT2:
                    doSendFloats(entry.path, entry.data.twoFloats.f1, entry.data.twoFloats.f2);
                    break;
                case QueueMsgType::GENERIC:
                    doSendGeneric(entry.genericMsg);
                    break;
            }
            
            entry.ready.store(false, std::memory_order_release);
            readPos = (readPos + 1) & QUEUE_MASK;
        }
        
        m_queueReadPos.store(readPos, std::memory_order_release);
    }
    
    LOG_INFO("OSC Sender-Thread beendet");
}

// ============================================================================
// Interne Send-Funktionen (nur vom Sender-Thread aufgerufen, kein Lock nötig)
// ============================================================================

void OscSender::doSendInts(const char* path, int i1, int i2, int i3) {
#ifdef HAS_LIBLO
    for (auto& target : m_targets) {
        if (!target.address) continue;
        lo_address addr = static_cast<lo_address>(target.address);
        lo_send(addr, path, "iii", i1, i2, i3);
    }
#else
    (void)path; (void)i1; (void)i2; (void)i3;
#endif
}

void OscSender::doSendFloat(const char* path, float f1) {
#ifdef HAS_LIBLO
    for (auto& target : m_targets) {
        if (!target.address) continue;
        lo_address addr = static_cast<lo_address>(target.address);
        lo_send(addr, path, "f", f1);
    }
#else
    (void)path; (void)f1;
#endif
}

void OscSender::doSendFloats(const char* path, float f1, float f2) {
#ifdef HAS_LIBLO
    for (auto& target : m_targets) {
        if (!target.address) continue;
        lo_address addr = static_cast<lo_address>(target.address);
        lo_send(addr, path, "ff", f1, f2);
    }
#else
    (void)path; (void)f1; (void)f2;
#endif
}

void OscSender::doSendGeneric(const OscMessage& msg) {
#ifdef HAS_LIBLO
    for (auto& target : m_targets) {
        if (!target.address) continue;
        
        lo_address addr = static_cast<lo_address>(target.address);
        lo_message lom = lo_message_new();
        
        for (const auto& arg : msg.args) {
            try {
                int intVal = std::stoi(arg);
                lo_message_add_int32(lom, intVal);
            } catch (...) {
                try {
                    float floatVal = std::stof(arg);
                    lo_message_add_float(lom, floatVal);
                } catch (...) {
                    lo_message_add_string(lom, arg.c_str());
                }
            }
        }
        
        lo_send_message(addr, msg.path.c_str(), lom);
        lo_message_free(lom);
    }
#else
    (void)msg;
#endif
}

} // namespace OSC
} // namespace BeatAnalyzer
