#pragma once

#include <string>
#include <vector>
#include <memory>

namespace BeatAnalyzer {
namespace OSC {

/**
 * OSC message building and sending utilities.
 */
struct OscMessage {
    std::string path;
    std::vector<std::string> args;
    
    OscMessage(const std::string& p = "") : path(p) {}
    
    void addInt(int value);
    void addFloat(float value);
    void addDouble(double value);
    void addString(const std::string& value);
    
    std::string toString() const;
};

/**
 * Beat clock message.
 * OSC Format: /beat iif  beat(1-4), bar, bpm
 */
struct BeatClockMessage {
    int track_id;           // Kanal-Index (0-based)
    int beat_number;        // 1-4 (Schlag im Takt)
    int bar_number;         // Takt-Nummer (ab 1, fortlaufend)
    double bpm;             // Aktuelles BPM (z.B. 128.53)
    
    OscMessage toOscMessage() const;
};

} // namespace OSC
} // namespace BeatAnalyzer
