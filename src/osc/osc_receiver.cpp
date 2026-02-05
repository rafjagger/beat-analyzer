#include "osc/osc_receiver.h"
#include "util/logging.h"
#include <chrono>

#ifdef HAS_LIBLO
#include <lo/lo.h>
#endif

namespace BeatAnalyzer {
namespace OSC {

OscReceiver::OscReceiver() = default;

OscReceiver::~OscReceiver() {
    stop();
}

void OscReceiver::setPort(int port) {
    m_port = port;
}

void OscReceiver::setBeatClockPath(const std::string& path) {
    m_beatClockPath = path;
}

void OscReceiver::setCallback(BeatClockCallback callback) {
    m_callback = callback;
}

bool OscReceiver::start() {
#ifdef HAS_LIBLO
    if (m_running) return true;
    
    std::string portStr = std::to_string(m_port);
    lo_server_thread st = lo_server_thread_new(portStr.c_str(), nullptr);
    
    if (!st) {
        LOG_ERROR("Failed to create OSC server on port " + portStr);
        return false;
    }
    
    m_serverThread = st;
    
    // Handler für Beat-Clock registrieren
    lo_server_thread_add_method(st, m_beatClockPath.c_str(), nullptr, 
                                 beatClockHandler, this);
    
    // Auch auf /beat/* hören
    lo_server_thread_add_method(st, "/beat", nullptr, beatClockHandler, this);
    
    lo_server_thread_start(st);
    m_running = true;
    
    LOG_INFO("OSC Receiver started on port " + portStr + 
             " (listening for " + m_beatClockPath + ")");
    return true;
#else
    LOG_WARN("OSC Receiver not available - liblo not compiled in");
    return false;
#endif
}

void OscReceiver::stop() {
#ifdef HAS_LIBLO
    if (!m_running) return;
    
    if (m_serverThread) {
        lo_server_thread_stop(static_cast<lo_server_thread>(m_serverThread));
        lo_server_thread_free(static_cast<lo_server_thread>(m_serverThread));
        m_serverThread = nullptr;
    }
    
    m_running = false;
    LOG_INFO("OSC Receiver stopped");
#endif
}

ReceivedBeatClock OscReceiver::getLastBeatClock() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_lastBeatClock;
}

int OscReceiver::beatClockHandler(const char* path, const char* types,
                                   void** argv, int argc, void* msg, void* userData) {
#ifdef HAS_LIBLO
    OscReceiver* self = static_cast<OscReceiver*>(userData);
    
    ReceivedBeatClock clock;
    clock.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    clock.valid = true;
    
    // Parse arguments - verschiedene Formate unterstützen
    if (argc >= 1 && types[0] == 'i') {
        clock.beatNumber = *static_cast<int32_t*>(argv[0]);
    }
    if (argc >= 2 && types[1] == 'f') {
        clock.bpm = *static_cast<float*>(argv[1]);
    }
    // Alternativ: nur BPM als float
    if (argc >= 1 && types[0] == 'f') {
        clock.bpm = *static_cast<float*>(argv[0]);
    }
    
    {
        std::lock_guard<std::mutex> lock(self->m_mutex);
        self->m_lastBeatClock = clock;
    }
    
    if (self->m_callback) {
        self->m_callback(clock);
    }
    
    return 0;
#else
    return 0;
#endif
}

} // namespace OSC
} // namespace BeatAnalyzer
