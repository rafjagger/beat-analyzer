#pragma once

#include <jack/jack.h>
#include <memory>
#include <vector>
#include <string>
#include <functional>
#include "audio_types.h"
#include "audio_buffer.h"

namespace BeatAnalyzer {
namespace Audio {

/**
 * JACK audio client for capturing multi-channel audio.
 * Supports 4 stereo channels (8 mono channels total).
 */
class JackClient {
public:
    using ProcessCallback = std::function<void(const CSAMPLE*, int)>;
    
    JackClient(const std::string& clientName = "beat-analyzer");
    ~JackClient();
    
    // Non-copyable
    JackClient(const JackClient&) = delete;
    JackClient& operator=(const JackClient&) = delete;
    
    // Initialize JACK client and connect to ports
    bool initialize();
    
    // Activate JACK client
    bool activate();
    
    // Deactivate JACK client
    bool deactivate();
    
    // Register callback for audio processing
    void setProcessCallback(ProcessCallback callback);
    
    // Get available JACK ports
    std::vector<std::string> getAvailablePorts(
        const std::string& portFilter = "");
    
    // Connect input port
    bool connectPort(int channelIndex, const std::string& portName);
    
    // Properties
    SampleRate getSampleRate() const;
    int getBufferSize() const;
    bool isConnected() const { return m_connected; }
    
    // Static JACK callbacks
    static int processCallback(jack_nframes_t nframes, void* arg);
    static void shutdownCallback(void* arg);
    
private:
    std::string m_clientName;
    jack_client_t* m_client;
    std::vector<jack_port_t*> m_inputPorts;
    std::vector<CSAMPLE*> m_portBuffers;
    ProcessCallback m_processCallback;
    bool m_connected;
    
    int m_processInternal(int frameCount);
};

using JackClientPtr = std::shared_ptr<JackClient>;

} // namespace Audio
} // namespace BeatAnalyzer
