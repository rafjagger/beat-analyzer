#pragma once

#include <vector>
#include <cmath>
#include <algorithm>

namespace BeatAnalyzer {

/**
 * VU Meter - berechnet RMS und Peak Level für Audio
 */
class VuMeter {
public:
    explicit VuMeter(int sampleRate = 44100);
    
    // Verarbeite Stereo-Audio und berechne Level
    void process(const float* stereoInput, int frameCount);
    
    // Getter für aktuelle Werte (in dB, 0 = max, negative = leiser)
    float getRmsDb() const { return m_rmsDb; }
    float getPeakDb() const { return m_peakDb; }
    
    // Getter für lineare Werte (0.0 - 1.0)
    float getRmsLinear() const { return m_rmsLinear; }
    float getPeakLinear() const { return m_peakLinear; }
    
    // Reset
    void reset();
    
private:
    int m_sampleRate;
    
    // RMS Berechnung
    float m_rmsSum;
    int m_rmsSamples;
    float m_rmsLinear;
    float m_rmsDb;
    
    // Peak mit Falloff
    float m_peakLinear;
    float m_peakDb;
    float m_peakHold;
    float m_peakFalloff;  // dB pro Sekunde
    
    // Konvertierung
    static float linearToDb(float linear);
    static float dbToLinear(float db);
};

} // namespace BeatAnalyzer
