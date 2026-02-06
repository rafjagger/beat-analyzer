#include "osc/osc_sender.h"
#include "util/logging.h"

#ifdef HAS_LIBLO
#include <lo/lo.h>
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
        target.address = lo_address_new(target.host.c_str(), 
                                        std::to_string(target.port).c_str());
        
        if (!target.address) {
            LOG_ERROR("Failed to create OSC address for " + target.name + ": " + 
                      target.host + ":" + std::to_string(target.port));
        } else {
            LOG_INFO("OSC target added: " + target.name + " -> " + 
                     target.host + ":" + std::to_string(target.port));
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
    
    bool allSuccess = true;
    for (const auto& target : m_targets) {
        if (!target.address) continue;
        
        lo_address addr = static_cast<lo_address>(target.address);
        int result = lo_send(addr, path.c_str(), "f", value);
        if (result < 0) allSuccess = false;
    }
    return allSuccess;
#else
    return false;
#endif
}

bool OscSender::sendFloats(const std::string& path, float value1, float value2) {
#ifdef HAS_LIBLO
    if (!m_connected) return false;
    
    bool allSuccess = true;
    for (const auto& target : m_targets) {
        if (!target.address) continue;
        
        lo_address addr = static_cast<lo_address>(target.address);
        int result = lo_send(addr, path.c_str(), "ff", value1, value2);
        if (result < 0) allSuccess = false;
    }
    return allSuccess;
#else
    return false;
#endif
}

bool OscSender::sendMessage(const OscMessage& msg) {
#ifdef HAS_LIBLO
    if (!m_connected) return false;
    
    bool allSuccess = true;
    
    for (const auto& target : m_targets) {
        if (!target.address) continue;
        
        lo_address addr = static_cast<lo_address>(target.address);
        
        // Build OSC message
        lo_message lom = lo_message_new();
        
        for (const auto& arg : msg.args) {
            // Try to determine type
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
        
        int result = lo_send_message(addr, msg.path.c_str(), lom);
        lo_message_free(lom);
        
        if (result < 0) allSuccess = false;
    }
    
    return allSuccess;
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
