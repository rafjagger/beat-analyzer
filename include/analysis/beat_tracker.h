#pragma once

#include <vector>
#include <memory>
#include "beat_info.h"
#include "audio/audio_types.h"

namespace BeatAnalyzer {
namespace Analysis {

/**
 * Beat tracker for maintaining beat grid and timing information.
 * Converts detected beats into playable beat clock.
 */
class BeatTracker {
public:
    explicit BeatTracker(SampleRate sampleRate);
    ~BeatTracker() = default;
    
    // Non-copyable
    BeatTracker(const BeatTracker&) = delete;
    BeatTracker& operator=(const BeatTracker&) = delete;
    
    // Update with new beat information
    void updateBeats(const BeatInfo& beatInfo);
    
    // Get beat number (0-3 for 4/4 time)
    int getBeatNumber(FramePos framePos) const;
    
    // Get next beat position from current frame
    FramePos getNextBeat(FramePos fromFrame) const;
    
    // Get previous beat position from current frame
    FramePos getPreviousBeat(FramePos fromFrame) const;
    
    // Get current tempo
    Bpm getCurrentBpm() const { return m_currentBpm; }
    
    // Get beat grid position (0.0-1.0 within beat)
    double getBeatPhase(FramePos framePos) const;
    
    // Check if position is close to a beat
    bool isNearBeat(FramePos framePos, double toleranceMs = 50.0) const;
    
    // Reset tracker
    void reset();
    
private:
    SampleRate m_sampleRate;
    std::vector<FramePos> m_beats;
    Bpm m_currentBpm;
    FramePos m_lastBeatPos;
    int m_beatsPerBar;  // Default 4 for 4/4 time
};

} // namespace Analysis
} // namespace BeatAnalyzer
