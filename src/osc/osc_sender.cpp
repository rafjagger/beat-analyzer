#include "osc/osc_sender.h"
#include "util/logging.h"

#ifdef HAS_LIBLO
#include <lo/lo.h>
#endif

namespace BeatAnalyzer {
namespace OSC {

OscSender::OscSender(const std::string& targetHost, int targetPort)
    : m_targetHost(targetHost),
      m_targetPort(targetPort),
      m_address(nullptr),
      m_connected(false) {
}

OscSender::~OscSender() {
#ifdef HAS_LIBLO
    if (m_address) {
        lo_address_free(static_cast<lo_address>(m_address));
    }
#endif
}

bool OscSender::initialize() {
#ifdef HAS_LIBLO
    m_address = lo_address_new(m_targetHost.c_str(), 
                               std::to_string(m_targetPort).c_str());
    
    if (!m_address) {
        LOG_ERROR("Failed to create OSC address: " + m_targetHost + ":" + 
                  std::to_string(m_targetPort));
        return false;
    }
    
    m_connected = true;
    LOG_INFO("OSC sender initialized: " + m_targetHost + ":" + 
             std::to_string(m_targetPort));
    return true;
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
    if (!m_connected || !m_address) return false;
    
    lo_address addr = static_cast<lo_address>(m_address);
    int result = lo_send(addr, path.c_str(), "f", value);
    return result >= 0;
#else
    return false;
#endif
}

bool OscSender::sendMessage(const OscMessage& msg) {
#ifdef HAS_LIBLO
    if (!m_connected || !m_address) return false;
    
    lo_address addr = static_cast<lo_address>(m_address);
    
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
    
    return result >= 0;
#else
    return false;
#endif
}

bool OscSender::broadcastBeatClock(
    int64_t framePos,
    float bpm,
    int beatNumber,
    float strength) {
    
    for (int track = 0; track < 4; ++track) {
        BeatClockMessage msg;
        msg.track_id = track;
        msg.frame_position = framePos;
        msg.bpm = bpm;
        msg.beat_number = beatNumber;
        msg.beat_strength = strength;
        
        if (!sendBeatClock(msg)) {
            return false;
        }
    }
    
    return true;
}

} // namespace OSC
} // namespace BeatAnalyzer
