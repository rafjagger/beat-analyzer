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
    
    // Handler für Clock-Modus
    lo_server_thread_add_method(st, "/clockmode", "i", clockModeHandler, this);
    lo_server_thread_add_method(st, "/clockmode", "f", clockModeHandler, this);
    
    // Handler für Tap (Downbeat setzen)
    lo_server_thread_add_method(st, "/tap", "i", tapHandler, this);
    lo_server_thread_add_method(st, "/tap", nullptr, tapHandler, this);
    
    lo_server_thread_start(st);
    m_running = true;
    
    LOG_INFO("OSC Receiver started on port " + portStr + 
             " (listening for " + m_beatClockPath + ", /clockmode, /tap)");
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

int OscReceiver::beatClockHandler(const char* /*path*/, const char* types,
                                   lo_arg** argv, int argc, lo_message /*msg*/, void* userData) {
#ifdef HAS_LIBLO
    OscReceiver* self = static_cast<OscReceiver*>(userData);
    
    ReceivedBeatClock clock;
    clock.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    clock.valid = true;
    
    // Parse: /beat iii  beat(1-4), bar, bpm
    if (argc >= 1 && types[0] == 'i') {
        clock.beatNumber = argv[0]->i;
    }
    if (argc >= 2 && types[1] == 'i') {
        clock.bar = argv[1]->i;
    }
    if (argc >= 3 && types[2] == 'i') {
        clock.bpm = argv[2]->i;
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
    (void)types; (void)argv; (void)argc; (void)userData;
    return 0;
#endif
}

int OscReceiver::clockModeHandler(const char* /*path*/, const char* types,
                                   lo_arg** argv, int argc, lo_message /*msg*/, void* userData) {
#ifdef HAS_LIBLO
    OscReceiver* self = static_cast<OscReceiver*>(userData);
    
    int mode = 0;
    
    if (argc >= 1) {
        if (types && types[0] == 'i') {
            mode = argv[0]->i;
        } else if (types && types[0] == 'f') {
            mode = static_cast<int>(argv[0]->f);
        }
    }
    
    // Nur 0 oder 1 erlaubt
    mode = (mode != 0) ? 1 : 0;
    
    int oldMode = self->m_clockMode.exchange(mode);
    
    if (oldMode != mode) {
        LOG_INFO("Clock-Modus gewechselt: " + std::string(mode ? "EIGENE BEATCLOCK" : "TRAINING"));
    }
    
    if (self->m_clockModeCallback) {
        self->m_clockModeCallback(mode);
    }
    
    return 0;
#else
    return 0;
#endif
}

int OscReceiver::tapHandler(const char* /*path*/, const char* types,
                             lo_arg** argv, int argc, lo_message /*msg*/, void* userData) {
#ifdef HAS_LIBLO
    OscReceiver* self = static_cast<OscReceiver*>(userData);
    
    int beat = 1;  // Default: Downbeat
    if (argc >= 1 && types && types[0] == 'i') {
        beat = argv[0]->i;
    }
    
    // Beat auf 1-4 clampen
    if (beat < 1 || beat > 4) beat = 1;
    
    LOG_INFO("TAP empfangen -> Beat " + std::to_string(beat));
    
    if (self->m_tapCallback) {
        self->m_tapCallback(beat);
    }
    
    return 0;
#else
    return 0;
#endif
}

} // namespace OSC
} // namespace BeatAnalyzer
