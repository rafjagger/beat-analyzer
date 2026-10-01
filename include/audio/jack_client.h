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
 * Supports separate BPM channels (beat detection) and VU channels (metering).
 */
class JackClient {
public:
    using ProcessCallback = std::function<void(const CSAMPLE*, int)>;
    // Callback für separate Kanal-Typen
    using MonoProcessCallback = std::function<void(
        const std::vector<const CSAMPLE*>& bpmBuffers,
        const std::vector<const CSAMPLE*>& vuBuffers,
        int frameCount)>;
    
    JackClient(const std::string& clientName = "beat-analyzer", 
               int numBpmChannels = 4, 
               int numVuChannels = 2);
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
    bool connectBpmPort(int channelIndex, const std::string& portName);
    bool connectVuPort(int channelIndex, const std::string& portName);
    
    // Properties
    SampleRate getSampleRate() const;
    int getBufferSize() const;
    int getNumBpmChannels() const { return m_numBpmChannels; }
    int getNumVuChannels() const { return m_numVuChannels; }
    bool isConnected() const { return m_connected; }
    
    // Static JACK callbacks
    static int processCallback(jack_nframes_t nframes, void* arg);
    static void shutdownCallback(void* arg);
    
private:
    std::string m_clientName;
    int m_numBpmChannels;
    int m_numVuChannels;
    jack_client_t* m_client;
    std::vector<jack_port_t*> m_bpmPorts;
    std::vector<jack_port_t*> m_vuPorts;
    MonoProcessCallback m_monoProcessCallback;
    bool m_connected;
    
    int m_processInternal(int frameCount);
};

using JackClientPtr = std::shared_ptr<JackClient>;

} // namespace Audio
} // namespace BeatAnalyzer
