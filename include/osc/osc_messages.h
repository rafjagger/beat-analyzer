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
 * Beat clock message for Pro Tools sync.
 */
struct BeatClockMessage {
    int track_id;           // 0-3 for 4 tracks
    int64_t frame_position; // Absolute frame position
    float bpm;              // Current BPM
    int beat_number;        // 0-3 for 4/4 time
    float beat_strength;    // 0.0-1.0
    
    OscMessage toOscMessage() const;
};

} // namespace OSC
} // namespace BeatAnalyzer
