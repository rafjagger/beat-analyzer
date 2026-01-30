#pragma once

#include <string>
#include <memory>
#include <functional>
#include "osc_messages.h"

namespace BeatAnalyzer {
namespace OSC {

/**
 * OSC sender using liblo.
 * Sends beat clock and synchronization messages to Pro Tools.
 */
class OscSender {
public:
    OscSender(const std::string& targetHost, int targetPort);
    ~OscSender();
    
    // Non-copyable
    OscSender(const OscSender&) = delete;
    OscSender& operator=(const OscSender&) = delete;
    
    // Initialize OSC sender
    bool initialize();
    
    // Send beat clock message
    bool sendBeatClock(const BeatClockMessage& msg);
    
    // Send raw OSC message
    bool sendMessage(const OscMessage& msg);
    
    // Send to all tracks (0-3)
    bool broadcastBeatClock(
        int64_t framePos,
        float bpm,
        int beatNumber,
        float strength);
    
    // Connection properties
    bool isConnected() const { return m_connected; }
    const std::string& getHost() const { return m_targetHost; }
    int getPort() const { return m_targetPort; }
    
    // Set error callback for debugging
    using ErrorCallback = std::function<void(const std::string&)>;
    void setErrorCallback(ErrorCallback callback) {
        m_errorCallback = callback;
    }
    
private:
    std::string m_targetHost;
    int m_targetPort;
    void* m_address;  // lo_address from liblo
    bool m_connected;
    ErrorCallback m_errorCallback;
};

using OscSenderPtr = std::shared_ptr<OscSender>;

} // namespace OSC
} // namespace BeatAnalyzer
