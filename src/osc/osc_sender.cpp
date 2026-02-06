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
}

OscSender::~OscSender() {
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
    return anyConnected;
#else
    LOG_WARN("liblo not available - OSC support disabled");
    m_connected = false;
    return false;
#endif
}

bool OscSender::sendBeatClock(const BeatClockMessage& msg) {
    return sendMessage(msg.toOscMessage());
}

bool OscSender::sendFloat(const std::string& path, float value) {
#ifdef HAS_LIBLO
    if (!m_connected) return false;
    
    for (auto& target : m_targets) {
        if (!target.address) continue;
        
        lo_address addr = static_cast<lo_address>(target.address);
        lo_send(addr, path.c_str(), "f", value);
        // UDP: Fehler ignorieren - fire and forget
    }
    return true;
#else
    return false;
#endif
}

bool OscSender::sendFloats(const std::string& path, float value1, float value2) {
#ifdef HAS_LIBLO
    if (!m_connected) return false;
    
    for (auto& target : m_targets) {
        if (!target.address) continue;
        
        lo_address addr = static_cast<lo_address>(target.address);
        lo_send(addr, path.c_str(), "ff", value1, value2);
        // UDP: Fehler ignorieren - fire and forget
    }
    return true;
#else
    return false;
#endif
}

bool OscSender::sendMessage(const OscMessage& msg) {
#ifdef HAS_LIBLO
    if (!m_connected) return false;
    
    for (auto& target : m_targets) {
        if (!target.address) continue;
        
        lo_address addr = static_cast<lo_address>(target.address);
        
        // Build OSC message
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
        // UDP: Fehler ignorieren - fire and forget
    }
    
    return true;
#else
    return false;
#endif
}

bool OscSender::broadcastBeatClock(
    int64_t framePos,
    float bpm,
    int beatNumber,
    float strength) {
    
    BeatClockMessage msg;
    msg.track_id = 0;
    msg.beat_number = beatNumber;
    msg.bar_number = 1;
    msg.bpm = static_cast<int>(bpm + 0.5f);
    
    return sendBeatClock(msg);
}

} // namespace OSC
} // namespace BeatAnalyzer
