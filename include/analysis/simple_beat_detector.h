#ifndef SIMPLE_BEAT_DETECTOR_H
#define SIMPLE_BEAT_DETECTOR_H

#include <cmath>

namespace BeatAnalyzer {

/**
 * Simple, robust beat detector based on musicdsp.org algorithm.
 * 
 * Algorithm:
 * 1. 2nd order low-pass filter (150 Hz) - isolates bass/kick
 * 2. Peak envelope follower with fast attack, slow release
 * 3. Schmitt trigger with hysteresis
 * 4. Rising edge detector
 * 
 * This works sample-by-sample and is very reliable for kick detection.
 */
class SimpleBeatDetector {
public:
    SimpleBeatDetector(float sampleRate = 44100.0f) {
        setSampleRate(sampleRate);
        reset();
    }
    
    void setSampleRate(float sampleRate) {
        m_sampleRate = sampleRate;
        
        // Low-pass filter for bass (150 Hz cutoff)
        const float FREQ_LP_BEAT = 150.0f;
        const float T_FILTER = 1.0f / (2.0f * M_PI * FREQ_LP_BEAT);
        m_filterCoeff = 1.0f / (sampleRate * T_FILTER);
        
        // Envelope release time (20ms)
        const float BEAT_RTIME = 0.02f;
        m_envRelease = std::exp(-1.0f / (sampleRate * BEAT_RTIME));
    }
    
    void reset() {
        m_filter1Out = 0.0f;
        m_filter2Out = 0.0f;
        m_peakEnv = 0.0f;
        m_beatTrigger = false;
        m_prevBeatPulse = false;
        m_beatPulse = false;
        m_currentEnvelope = 0.0f;
    }
    
    /**
     * Process a single audio sample.
     * Call this for every sample.
     * Returns true on beat detection (one sample pulse).
     */
    bool process(float input) {
        // Step 1: 2nd order low-pass filter (two cascaded 1st order RC filters)
        m_filter1Out += m_filterCoeff * (input - m_filter1Out);
        m_filter2Out += m_filterCoeff * (m_filter1Out - m_filter2Out);
        
        // Step 2: Peak envelope follower
        float envIn = std::fabs(m_filter2Out);
        if (envIn > m_peakEnv) {
            // Instant attack
            m_peakEnv = envIn;
        } else {
            // Slow release
            m_peakEnv = m_peakEnv * m_envRelease + (1.0f - m_envRelease) * envIn;
        }
        
        m_currentEnvelope = m_peakEnv;
        
        // Step 3: Schmitt trigger with hysteresis
        // Thresholds are adaptive based on running peak
        if (!m_beatTrigger) {
            if (m_peakEnv > m_triggerHigh) {
                m_beatTrigger = true;
            }
        } else {
            if (m_peakEnv < m_triggerLow) {
                m_beatTrigger = false;
            }
        }
        
        // Step 4: Rising edge detector (one-sample pulse)
        m_beatPulse = false;
        if (m_beatTrigger && !m_prevBeatPulse) {
            m_beatPulse = true;
        }
        m_prevBeatPulse = m_beatTrigger;
        
        return m_beatPulse;
    }
    
    /**
     * Process a block of samples.
     * Returns true if at least one beat was detected.
     * onsetStrength is set to the envelope value at the strongest beat.
     */
    bool processBlock(const float* samples, int numSamples, float& onsetStrength) {
        bool beatDetected = false;
        float maxEnvAtBeat = 0.0f;
        
        for (int i = 0; i < numSamples; ++i) {
            if (process(samples[i])) {
                beatDetected = true;
                if (m_currentEnvelope > maxEnvAtBeat) {
                    maxEnvAtBeat = m_currentEnvelope;
                }
            }
        }
        
        onsetStrength = maxEnvAtBeat * 1000.0f;  // Scale for visibility
        return beatDetected;
    }
    
    /**
     * Set trigger thresholds.
     * Default: high=0.3, low=0.15 (original musicdsp values)
     * For louder signals, increase these.
     */
    void setThresholds(float high, float low) {
        m_triggerHigh = high;
        m_triggerLow = low;
    }
    
    /**
     * Set thresholds as a ratio of running peak.
     * This makes detection adaptive to input level.
     */
    void setAdaptiveThreshold(float highRatio, float lowRatio) {
        // Will be implemented with running peak tracking
        m_adaptiveHighRatio = highRatio;
        m_adaptiveLowRatio = lowRatio;
        m_useAdaptive = true;
    }
    
    float getCurrentEnvelope() const { return m_currentEnvelope; }
    bool isBeatTriggerActive() const { return m_beatTrigger; }
    
private:
    float m_sampleRate = 44100.0f;
    
    // Filter state
    float m_filterCoeff = 0.0f;
    float m_filter1Out = 0.0f;
    float m_filter2Out = 0.0f;
    
    // Envelope follower
    float m_envRelease = 0.0f;
    float m_peakEnv = 0.0f;
    float m_currentEnvelope = 0.0f;
    
    // Schmitt trigger
    float m_triggerHigh = 0.02f;   // Lowered from 0.3 for real audio
    float m_triggerLow = 0.01f;    // Lowered from 0.15
    bool m_beatTrigger = false;
    
    // Rising edge detector
    bool m_prevBeatPulse = false;
    bool m_beatPulse = false;
    
    // Adaptive threshold (optional)
    bool m_useAdaptive = false;
    float m_adaptiveHighRatio = 0.6f;
    float m_adaptiveLowRatio = 0.3f;
};

/**
 * Adaptive version that auto-adjusts thresholds based on signal level.
 */
class AdaptiveBeatDetector {
public:
    AdaptiveBeatDetector(float sampleRate = 44100.0f) 
        : m_detector(sampleRate), m_sampleRate(sampleRate) {
        reset();
    }
    
    void setSampleRate(float sampleRate) {
        m_sampleRate = sampleRate;
        m_detector.setSampleRate(sampleRate);
        
        // Running peak decay: ~2 seconds
        m_peakDecay = std::exp(-1.0f / (sampleRate * 2.0f));
    }
    
    void reset() {
        m_detector.reset();
        m_runningPeak = 0.0f;
        m_lastBeatSample = 0;
        m_currentSample = 0;
        m_minBeatIntervalSamples = static_cast<int>(m_sampleRate * 0.1f);  // 100ms = 600 BPM max
    }
    
    bool process(float input) {
        m_currentSample++;
        
        // First process the sample
        bool beat = m_detector.process(input);
        
        // Then track running peak of envelope (AFTER processing)
        float env = m_detector.getCurrentEnvelope();
        if (env > m_runningPeak) {
            m_runningPeak = env;
        } else {
            m_runningPeak *= m_peakDecay;
        }
        
        // Adaptive thresholds
        float adaptiveHigh = m_runningPeak * 0.5f;
        float adaptiveLow = m_runningPeak * 0.25f;
        
        // Minimum thresholds
        if (adaptiveHigh < 0.001f) adaptiveHigh = 0.001f;
        if (adaptiveLow < 0.0005f) adaptiveLow = 0.0005f;
        
        m_detector.setThresholds(adaptiveHigh, adaptiveLow);
        
        // Debug every 10000 samples
        static int debugCounter = 0;
        if (++debugCounter >= 10000) {
            printf("SIMPLE | env=%.6f peak=%.6f thrH=%.6f thrL=%.6f trig=%d\n",
                   env, m_runningPeak, adaptiveHigh, adaptiveLow, 
                   m_detector.isBeatTriggerActive() ? 1 : 0);
            fflush(stdout);
            debugCounter = 0;
        }
        
        // Minimum interval check (prevent double triggers)
        if (beat) {
            int interval = m_currentSample - m_lastBeatSample;
            if (interval < m_minBeatIntervalSamples) {
                beat = false;  // Too soon, ignore
            } else {
                m_lastBeatSample = m_currentSample;
                m_lastBeatStrength = env * 1000.0f;
            }
        }
        
        return beat;
    }
    
    bool processBlock(const float* samples, int numSamples, float& onsetStrength) {
        bool beatDetected = false;
        float maxStrength = 0.0f;
        
        for (int i = 0; i < numSamples; ++i) {
            if (process(samples[i])) {
                beatDetected = true;
                if (m_lastBeatStrength > maxStrength) {
                    maxStrength = m_lastBeatStrength;
                }
            }
        }
        
        onsetStrength = maxStrength;
        return beatDetected;
    }
    
    float getLastBeatStrength() const { return m_lastBeatStrength; }
    
private:
    SimpleBeatDetector m_detector;
    float m_sampleRate = 44100.0f;
    float m_runningPeak = 0.0f;
    float m_peakDecay = 0.0f;
    int m_lastBeatSample = 0;
    int m_currentSample = 0;
    int m_minBeatIntervalSamples = 4410;  // 100ms at 44100
    float m_lastBeatStrength = 0.0f;
};

} // namespace BeatAnalyzer

#endif // SIMPLE_BEAT_DETECTOR_H
