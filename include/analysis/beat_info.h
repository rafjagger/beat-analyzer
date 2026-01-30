#pragma once

#include <vector>
#include "audio/audio_types.h"

namespace BeatAnalyzer {
namespace Analysis {

using namespace BeatAnalyzer::Audio;

/**
 * Beat information structure.
 */
struct BeatInfo {
    std::vector<FramePos> beat_frames;  // Detected beat positions in sample frames
    Bpm bpm;                             // Estimated BPM
    double confidence;                   // Confidence 0.0-1.0
    bool valid;                          // Data validity flag
    
    BeatInfo()
        : bpm(0.0), confidence(0.0), valid(false) {}
    
    BeatInfo(const std::vector<FramePos>& frames, Bpm b, double conf)
        : beat_frames(frames), bpm(b), confidence(conf), valid(b.isValid()) {}
};

} // namespace Analysis
} // namespace BeatAnalyzer
