#include "audio/jack_client.h"
#include "util/logging.h"
#include <algorithm>
#include <jack/jack.h>

namespace BeatAnalyzer {
namespace Audio {

JackClient::JackClient(const std::string& clientName, int numChannels)
    : m_clientName(clientName),
      m_numChannels(std::max(1, std::min(8, numChannels))),
      m_client(nullptr),
      m_connected(false) {
}

JackClient::~JackClient() {
    if (m_client) {
        jack_client_close(m_client);
    }
}

bool JackClient::initialize() {
    jack_options_t options = JackNullOption;
    jack_status_t status;
    
    m_client = jack_client_open(m_clientName.c_str(), options, &status);
    if (!m_client) {
        LOG_ERROR("Failed to open JACK client");
        return false;
    }
    
    // Create mono input ports
    for (int i = 0; i < m_numChannels; ++i) {
        std::string portName = "in_" + std::to_string(i + 1);
        jack_port_t* port = jack_port_register(
            m_client,
            portName.c_str(),
            JACK_DEFAULT_AUDIO_TYPE,
            JackPortIsInput,
            0);
        
        if (!port) {
            LOG_ERROR("Failed to create JACK port: " + portName);
            return false;
        }
        m_inputPorts.push_back(port);
    }
    
    // Set process callback
    jack_set_process_callback(m_client, processCallback, this);
    jack_on_shutdown(m_client, shutdownCallback, this);
    
    m_connected = true;
    LOG_INFO("JACK client initialized with " + std::to_string(m_numChannels) + " mono channels");
    return true;
}

bool JackClient::activate() {
    if (!m_client) return false;
    
    if (jack_activate(m_client)) {
        LOG_ERROR("Failed to activate JACK client");
        return false;
    }
    
    LOG_INFO("JACK client activated");
    return true;
}

bool JackClient::deactivate() {
    if (!m_client) return false;
    
    if (jack_deactivate(m_client)) {
        LOG_ERROR("Failed to deactivate JACK client");
        return false;
    }
    
    LOG_INFO("JACK client deactivated");
    return true;
}

void JackClient::setMonoProcessCallback(MonoProcessCallback callback) {
    m_monoProcessCallback = callback;
}

SampleRate JackClient::getSampleRate() const {
    if (!m_client) return SampleRate(44100);
    return SampleRate(jack_get_sample_rate(m_client));
}

int JackClient::getBufferSize() const {
    if (!m_client) return 512;
    return jack_get_buffer_size(m_client);
}

std::vector<std::string> JackClient::getAvailablePorts(
    const std::string& portFilter) {
    std::vector<std::string> ports;
    if (!m_client) return ports;
    
    const char** jackPorts = jack_get_ports(
        m_client,
        portFilter.empty() ? nullptr : portFilter.c_str(),
        JACK_DEFAULT_AUDIO_TYPE,
        JackPortIsOutput | JackPortIsPhysical);
    
    if (jackPorts) {
        for (int i = 0; jackPorts[i]; ++i) {
            ports.push_back(jackPorts[i]);
        }
        jack_free(jackPorts);
    }
    
    return ports;
}

bool JackClient::connectPort(int channelIndex, const std::string& portName) {
    if (channelIndex >= static_cast<int>(m_inputPorts.size())) {
        return false;
    }
    
    std::string ourPort = m_clientName + ":in_" + std::to_string(channelIndex + 1);
    
    if (jack_connect(m_client, portName.c_str(), ourPort.c_str())) {
        LOG_ERROR("Failed to connect port: " + portName);
        return false;
    }
    
    LOG_INFO("Connected port: " + portName);
    return true;
}

int JackClient::processCallback(jack_nframes_t nframes, void* arg) {
    JackClient* client = static_cast<JackClient*>(arg);
    return client->m_processInternal(nframes);
}

void JackClient::shutdownCallback(void* arg) {
    JackClient* client = static_cast<JackClient*>(arg);
    client->m_connected = false;
    LOG_WARN("JACK server shutdown");
}

int JackClient::m_processInternal(int frameCount) {
    // Get audio buffers from all mono ports
    std::vector<const CSAMPLE*> monoBuffers;
    for (size_t i = 0; i < m_inputPorts.size(); ++i) {
        CSAMPLE* buf = static_cast<CSAMPLE*>(
            jack_port_get_buffer(m_inputPorts[i], frameCount));
        monoBuffers.push_back(buf);
    }
    
    // Call mono callback
    if (m_monoProcessCallback) {
        m_monoProcessCallback(monoBuffers, frameCount);
    }
    
    return 0;
}

} // namespace Audio
} // namespace BeatAnalyzer
