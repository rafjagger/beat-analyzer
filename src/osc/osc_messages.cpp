#include "osc/osc_messages.h"
#include <sstream>
#include <iomanip>

namespace BeatAnalyzer {
namespace OSC {

void OscMessage::addInt(int value) {
    args.push_back(std::to_string(value));
}

void OscMessage::addFloat(float value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << value;
    args.push_back(oss.str());
}

void OscMessage::addDouble(double value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << value;
    args.push_back(oss.str());
}

void OscMessage::addString(const std::string& value) {
    args.push_back(value);
}

std::string OscMessage::toString() const {
    std::ostringstream oss;
    oss << path;
    for (const auto& arg : args) {
        oss << " " << arg;
    }
    return oss.str();
}

OscMessage BeatClockMessage::toOscMessage() const {
    // Track-ID wird Teil des Pfades: /beatclock/1, /beatclock/2, /beatclock/3, /beatclock/4
    OscMessage msg("/beatclock/" + std::to_string(track_id + 1));
    msg.addInt(static_cast<int>(frame_position));
    msg.addFloat(bpm);
    msg.addInt(beat_number);
    msg.addFloat(beat_strength);
    return msg;
}

} // namespace OSC
} // namespace BeatAnalyzer
