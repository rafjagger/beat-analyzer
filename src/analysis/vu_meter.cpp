#include "analysis/vu_meter.h"
#include <cmath>
#include <algorithm>

namespace BeatAnalyzer {

VuMeter::VuMeter(int sampleRate)
    : m_sampleRate(sampleRate),
      m_rmsSum(0.0f),
      m_rmsSamples(0),
      m_rmsLinear(0.0f),
      m_rmsDb(-60.0f),
      m_rmsAttack(1.0f),
      m_rmsRelease(0.5f) {
}

void VuMeter::notePeak(float blockPeak) {
    // Raise the maximum unless the sender reset it meanwhile; then retry
    // against zero, so a reset never brings back the peak it just took.
    float current = m_peakSinceTake.load(std::memory_order_relaxed);
    while (blockPeak > current &&
           !m_peakSinceTake.compare_exchange_weak(current, blockPeak, std::memory_order_acq_rel)) {
    }
}

void VuMeter::followRms(float currentRms) {
    if (m_rmsAttack >= 1.0f) {
        m_rmsLinear = currentRms;
    } else if (currentRms > m_rmsLinear) {
        m_rmsLinear = m_rmsAttack * currentRms + (1.0f - m_rmsAttack) * m_rmsLinear;
    } else {
        m_rmsLinear = m_rmsRelease * currentRms + (1.0f - m_rmsRelease) * m_rmsLinear;
    }
}

// Schnelle Mono-Version (kein Stereo-Overhead)
void VuMeter::processMono(const float* monoInput, int frameCount) {
    float maxPeak = 0.0f;
    float sumSquares = 0.0f;
    
    for (int i = 0; i < frameCount; ++i) {
        float absVal = std::fabs(monoInput[i]);
        if (absVal > maxPeak) maxPeak = absVal;
        sumSquares += monoInput[i] * monoInput[i];
    }
    
    followRms(std::sqrt(sumSquares / frameCount));
    notePeak(maxPeak);
}

void VuMeter::process(const float* stereoInput, int frameCount) {
    float maxPeak = 0.0f;
    float sumSquares = 0.0f;
    
    for (int i = 0; i < frameCount; ++i) {
        float left = stereoInput[i * 2];
        float right = stereoInput[i * 2 + 1];
        float mono = (left + right) * 0.5f;
        
        float absVal = std::fabs(mono);
        if (absVal > maxPeak) maxPeak = absVal;
        sumSquares += mono * mono;
    }
    
    followRms(std::sqrt(sumSquares / frameCount));
    notePeak(maxPeak);
}

void VuMeter::reset() {
    m_rmsSum = 0.0f;
    m_rmsSamples = 0;
    m_rmsLinear = 0.0f;
    m_rmsDb = -60.0f;
    m_peakSinceTake.store(0.0f, std::memory_order_relaxed);
}

float VuMeter::linearToDb(float linear) {
    if (linear <= 0.0001f) return -60.0f;
    return 20.0f * std::log10(linear);
}

float VuMeter::dbToLinear(float db) {
    if (db <= -60.0f) return 0.0f;
    return std::pow(10.0f, db / 20.0f);
}

} // namespace BeatAnalyzer
