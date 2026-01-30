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
      m_peakLinear(0.0f),
      m_peakDb(-60.0f),
      m_peakHold(0.0f),
      m_peakFalloff(20.0f) {  // 20 dB/s Falloff
}

void VuMeter::process(const float* stereoInput, int frameCount) {
    float maxPeak = m_peakLinear;
    float sumSquares = 0.0f;
    
    // Verarbeite Stereo zu Mono und berechne RMS + Peak
    for (int i = 0; i < frameCount; ++i) {
        // Mono downmix (L + R) / 2
        float left = stereoInput[i * 2];
        float right = stereoInput[i * 2 + 1];
        float mono = (left + right) * 0.5f;
        
        // Peak (Maximum der absoluten Werte)
        float absVal = std::fabs(mono);
        if (absVal > maxPeak) {
            maxPeak = absVal;
        }
        
        // RMS (Summe der Quadrate)
        sumSquares += mono * mono;
    }
    
    // RMS berechnen (gleitender Durchschnitt über ~50ms)
    int rmsWindow = m_sampleRate / 20;  // 50ms bei 44100 Hz = 2205 samples
    m_rmsSum += sumSquares;
    m_rmsSamples += frameCount;
    
    if (m_rmsSamples >= rmsWindow) {
        m_rmsLinear = std::sqrt(m_rmsSum / m_rmsSamples);
        m_rmsDb = linearToDb(m_rmsLinear);
        m_rmsSum = 0.0f;
        m_rmsSamples = 0;
    }
    
    // Peak mit Falloff
    float falloffPerFrame = m_peakFalloff * frameCount / m_sampleRate;
    float falloffLinear = dbToLinear(m_peakDb - falloffPerFrame);
    
    if (maxPeak > m_peakLinear) {
        // Neuer Peak
        m_peakLinear = maxPeak;
        m_peakDb = linearToDb(maxPeak);
    } else {
        // Falloff
        m_peakLinear = std::max(falloffLinear, 0.0001f);
        m_peakDb = linearToDb(m_peakLinear);
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
