#pragma once

#include <string>
#include <memory>
#include <functional>
#include <vector>
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
 * Sends beat clock and synchronization messages to all configured hosts.
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
    
    // Initialize all OSC targets
    bool initialize();
    
    // Send beat clock message to all targets
    bool sendBeatClock(const BeatClockMessage& msg);
    
    // Send single float value to path (all targets)
    bool sendFloat(const std::string& path, float value);
    
    // Send raw OSC message to all targets
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
};

using OscSenderPtr = std::shared_ptr<OscSender>;

} // namespace OSC
} // namespace BeatAnalyzer
