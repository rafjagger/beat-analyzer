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
    // /beat iii  beat(1-4), bar, bpm
    OscMessage msg("/beat");
    msg.addInt(beat_number);
    msg.addInt(bar_number);
    msg.addInt(bpm);
    return msg;
}

} // namespace OSC
} // namespace BeatAnalyzer
