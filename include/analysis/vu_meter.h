#pragma once

#include <atomic>
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
    
    // Schnelle Mono-Version (kein Stereo-Overhead)
    void processMono(const float* monoInput, int frameCount);
    
    // Getter für aktuelle Werte (in dB, 0 = max, negative = leiser)
    float getRmsDb() const { return m_rmsDb; }
    
    // Getter für lineare Werte (0.0 - 1.0)
    float getRmsLinear() const { return m_rmsLinear; }

    // The highest sample peak since the last call, raw: no fall, no hold.
    // The sender takes it once per tick; the displays apply the ballistics.
    // Safe against processMono() on the JACK thread.
    float takePeakLinear() { return m_peakSinceTake.exchange(0.0f, std::memory_order_acq_rel); }
    
    // Konfiguration
    void setRmsAttack(float attack) { m_rmsAttack = attack; }
    void setRmsRelease(float release) { m_rmsRelease = release; }
    
    // Reset
    void reset();
    
private:
    int m_sampleRate;
    
    // RMS Berechnung
    float m_rmsSum;
    int m_rmsSamples;
    float m_rmsLinear;
    float m_rmsDb;
    float m_rmsAttack;    // 0.0-1.0, höher = schneller
    float m_rmsRelease;   // 0.0-1.0, höher = schneller
    
    std::atomic<float> m_peakSinceTake{0.0f};

    void notePeak(float blockPeak);
    void followRms(float currentRms);
    
    // Konvertierung
    static float linearToDb(float linear);
    static float dbToLinear(float db);
};

} // namespace BeatAnalyzer
