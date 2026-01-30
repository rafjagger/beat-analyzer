#include "analysis/beat_tracker.h"
#include <algorithm>
#include <cmath>

namespace BeatAnalyzer {
namespace Analysis {

BeatTracker::BeatTracker(SampleRate sampleRate)
    : m_sampleRate(sampleRate),
      m_currentBpm(0.0),
      m_lastBeatPos(0),
      m_beatsPerBar(4) {
}

void BeatTracker::updateBeats(const BeatInfo& beatInfo) {
    m_beats = beatInfo.beat_frames;
    m_currentBpm = beatInfo.bpm;
    
    if (!m_beats.empty()) {
        m_lastBeatPos = m_beats.back();
    }
}

int BeatTracker::getBeatNumber(FramePos framePos) const {
    if (m_beats.empty() || !m_currentBpm.isValid()) {
        return 0;
    }
    
    // Find closest beat
    int closestIdx = 0;
    int minDist = std::abs(m_beats[0].frame - framePos.frame);
    
    for (size_t i = 1; i < m_beats.size(); ++i) {
        int dist = std::abs(m_beats[i].frame - framePos.frame);
        if (dist < minDist) {
            minDist = dist;
            closestIdx = i;
        }
    }
    
    return closestIdx % m_beatsPerBar;
}

FramePos BeatTracker::getNextBeat(FramePos fromFrame) const {
    if (m_beats.empty()) {
        return fromFrame;
    }
    
    for (const auto& beat : m_beats) {
        if (beat.frame > fromFrame.frame) {
            return beat;
        }
    }
    
    // Extrapolate based on BPM if needed
    if (m_currentBpm.isValid()) {
        double beatIntervalFrames = (60.0 / m_currentBpm.value) * m_sampleRate;
        return FramePos(
            static_cast<int64_t>(m_lastBeatPos.frame + beatIntervalFrames));
    }
    
    return m_lastBeatPos;
}

FramePos BeatTracker::getPreviousBeat(FramePos fromFrame) const {
    if (m_beats.empty()) {
        return fromFrame;
    }
    
    for (int i = static_cast<int>(m_beats.size()) - 1; i >= 0; --i) {
        if (m_beats[i].frame < fromFrame.frame) {
            return m_beats[i];
        }
    }
    
    return m_beats[0];
}

double BeatTracker::getBeatPhase(FramePos framePos) const {
    FramePos prevBeat = getPreviousBeat(framePos);
    FramePos nextBeat = getNextBeat(framePos);
    
    int64_t beatInterval = nextBeat.frame - prevBeat.frame;
    if (beatInterval <= 0) return 0.0;
    
    int64_t frameSinceBeat = framePos.frame - prevBeat.frame;
    return static_cast<double>(frameSinceBeat) / beatInterval;
}

bool BeatTracker::isNearBeat(FramePos framePos, double toleranceMs) const {
    double toleranceFrames = (toleranceMs / 1000.0) * m_sampleRate;
    
    FramePos nextBeat = getNextBeat(framePos);
    return std::abs(nextBeat.frame - framePos.frame) < toleranceFrames;
}

void BeatTracker::reset() {
    m_beats.clear();
    m_currentBpm = Bpm(0.0);
    m_lastBeatPos = FramePos(0);
}

} // namespace Analysis
} // namespace BeatAnalyzer
