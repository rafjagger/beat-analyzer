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
 * JACK audio client for capturing multi-channel mono audio.
 * Supports configurable number of mono channels (1-8).
 */
class JackClient {
public:
    using ProcessCallback = std::function<void(const CSAMPLE*, int)>;
    // Callback für separate Mono-Kanäle
    using MonoProcessCallback = std::function<void(const std::vector<const CSAMPLE*>&, int)>;
    
    JackClient(const std::string& clientName = "beat-analyzer", int numChannels = 4);
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
    
    // Register callback for mono channel processing
    void setMonoProcessCallback(MonoProcessCallback callback);
    
    // Get available JACK ports
    std::vector<std::string> getAvailablePorts(
        const std::string& portFilter = "");
    
    // Connect input port
    bool connectPort(int channelIndex, const std::string& portName);
    
    // Properties
    SampleRate getSampleRate() const;
    int getBufferSize() const;
    int getNumChannels() const { return m_numChannels; }
    bool isConnected() const { return m_connected; }
    
    // Static JACK callbacks
    static int processCallback(jack_nframes_t nframes, void* arg);
    static void shutdownCallback(void* arg);
    
private:
    std::string m_clientName;
    int m_numChannels;
    jack_client_t* m_client;
    std::vector<jack_port_t*> m_inputPorts;
    MonoProcessCallback m_monoProcessCallback;
    bool m_connected;
    
    int m_processInternal(int frameCount);
};

using JackClientPtr = std::shared_ptr<JackClient>;

} // namespace Audio
} // namespace BeatAnalyzer
