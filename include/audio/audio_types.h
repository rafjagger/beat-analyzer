#pragma once

#include <cstdint>
#include <vector>
#include <cmath>

namespace BeatAnalyzer {
namespace Audio {

// Audio sample type (float, same as JACK)
using CSAMPLE = float;

// Frame position in samples
struct FramePos {
    int64_t frame;
    
    explicit FramePos(int64_t f = 0) : frame(f) {}
    
    double toSeconds(int sampleRate) const {
        return static_cast<double>(frame) / sampleRate;
    }
    
    static FramePos fromSeconds(double seconds, int sampleRate) {
        return FramePos(static_cast<int64_t>(seconds * sampleRate));
    }
};

// Sample rate wrapper
struct SampleRate {
    int value;
    
    explicit SampleRate(int v = 44100) : value(v) {}
    
    operator int() const { return value; }
    bool operator==(int v) const { return value == v; }
};

// Channel count
struct ChannelCount {
    int value;
    
    explicit ChannelCount(int v = 2) : value(v) {}
    
    operator int() const { return value; }
    
    static ChannelCount mono() { return ChannelCount(1); }
    static ChannelCount stereo() { return ChannelCount(2); }
    static ChannelCount quad() { return ChannelCount(4); }
    static ChannelCount stem() { return ChannelCount(8); }
};

// Signal information
struct SignalInfo {
    SampleRate sampleRate;
    ChannelCount channels;
    
    SignalInfo(int sr = 44100, int ch = 2)
        : sampleRate(sr), channels(ch) {}
};

// BPM wrapper
struct Bpm {
    double value;
    
    Bpm(double v = 0.0) : value(v) {}
    
    operator double() const { return value; }
    bool isValid() const { return value > 0.0 && value < 300.0; }
};

} // namespace Audio
} // namespace BeatAnalyzer
