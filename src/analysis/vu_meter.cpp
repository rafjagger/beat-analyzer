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
      m_rmsRelease(0.5f),
      m_peakLinear(0.0f),
      m_peakDb(-60.0f),
      m_peakHold(0.0f),
      m_peakFalloff(40.0f) {
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
    
    // RMS direkt (keine Glättung bei Attack=1.0)
    float currentRms = std::sqrt(sumSquares / frameCount);
    
    if (m_rmsAttack >= 1.0f) {
        m_rmsLinear = currentRms;  // Sofort, keine Interpolation
    } else if (currentRms > m_rmsLinear) {
        m_rmsLinear = m_rmsAttack * currentRms + (1.0f - m_rmsAttack) * m_rmsLinear;
    } else {
        m_rmsLinear = m_rmsRelease * currentRms + (1.0f - m_rmsRelease) * m_rmsLinear;
    }
    
    // Peak sofort übernehmen
    if (maxPeak > m_peakLinear) {
        m_peakLinear = maxPeak;
    } else {
        // Falloff
        float falloff = m_peakFalloff * frameCount / m_sampleRate;
        m_peakLinear *= std::pow(10.0f, -falloff / 20.0f);
        if (m_peakLinear < 0.0001f) m_peakLinear = 0.0f;
    }
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
    
    float currentRms = std::sqrt(sumSquares / frameCount);
    
    if (m_rmsAttack >= 1.0f) {
        m_rmsLinear = currentRms;
    } else if (currentRms > m_rmsLinear) {
        m_rmsLinear = m_rmsAttack * currentRms + (1.0f - m_rmsAttack) * m_rmsLinear;
    } else {
        m_rmsLinear = m_rmsRelease * currentRms + (1.0f - m_rmsRelease) * m_rmsLinear;
    }
    
    if (maxPeak > m_peakLinear) {
        m_peakLinear = maxPeak;
    } else {
        float falloff = m_peakFalloff * frameCount / m_sampleRate;
        m_peakLinear *= std::pow(10.0f, -falloff / 20.0f);
        if (m_peakLinear < 0.0001f) m_peakLinear = 0.0f;
    }
}

void VuMeter::reset() {
    m_rmsSum = 0.0f;
    m_rmsSamples = 0;
    m_rmsLinear = 0.0f;
    m_rmsDb = -60.0f;
    m_peakLinear = 0.0f;
    m_peakDb = -60.0f;
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
