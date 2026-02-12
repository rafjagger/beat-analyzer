#pragma once

#include "BTrack.h"
#include <atomic>
#include <vector>

/**
 * BTrackWrapper - Minimaler Wrapper um BTrack
 * 
 * Exakt wie die Max External implementiert:
 *   hopSize = JACK Buffer Size (z.B. 128)
 *   frameSize = hopSize * 2 (z.B. 256)
 *   Jeder JACK-Callback = genau ein processAudioFrame() Aufruf
 * 
 * KEIN manuelles Frame-Sammeln, KEIN Overlap-Handling!
 * BTrack's ODF macht das Overlap intern.
 * 
 * Referenz: external/BTrack/plugins/max-external/btrack~/btrack~.cpp
 *   btrack_dsp64: hopSize = maxvectorsize; frameSize = hopSize * 2;
 *   btrack_perform64: processAudioFrame(audioFrame) pro Callback
 */
class BTrackWrapper {
public:
    /**
     * hopSize MUSS gleich JACK Buffer Size sein!
     * frameSize = hopSize * 2 (BTrack Standard)
     */
    BTrackWrapper(int hopSize, int frameSize = 0)
        : m_hopSize(hopSize)
        , m_frameSize(frameSize > 0 ? frameSize : hopSize * 2)
        , m_btrack(hopSize, m_frameSize)
        , m_beatDetected(false)
        , m_bpm(120.0)
        , m_audioFrame(hopSize, 0.0)
    {
    }

    /**
     * Einen JACK-Buffer verarbeiten.
     * numSamples MUSS == hopSize sein (= JACK Buffer Size).
     * Genau ein processAudioFrame() pro Aufruf, wie in der Max External.
     */
    bool processSamples(const float* samples, int numSamples) {
        // Float→Double Konversion (JACK liefert float, BTrack braucht double)
        for (int i = 0; i < numSamples; ++i) {
            m_audioFrame[i] = static_cast<double>(samples[i]);
        }
        
        // BTrack verarbeitet den Frame
        m_btrack.processAudioFrame(m_audioFrame.data());
        
        // Beat-Detection
        bool beatNow = m_btrack.beatDueInCurrentFrame();
        if (beatNow) {
            m_beatDetected.store(true, std::memory_order_release);
        }
        
        // BPM immer updaten
        m_bpm.store(m_btrack.getCurrentTempoEstimate(), std::memory_order_release);
        
        return beatNow;
    }

    /** Beat abholen (Flag wird zurückgesetzt) */
    bool hasBeatOccurred() {
        return m_beatDetected.exchange(false, std::memory_order_acq_rel);
    }

    /** BPM von BTrack (80-160 Bereich, BTrack-intern normalisiert) */
    double getBpm() const {
        return m_bpm.load(std::memory_order_acquire);
    }

    /** Tempo-Hint setzen (soft) */
    void setTempo(double bpm) { m_btrack.setTempo(bpm); }
    
    /** Tempo hard-locken */
    void fixTempo(double bpm) { m_btrack.fixTempo(bpm); }
    
    /** Tempo-Lock aufheben */
    void unfixTempo() { m_btrack.doNotFixTempo(); }

    int getHopSize() const { return m_hopSize; }
    int getFrameSize() const { return m_frameSize; }

private:
    int m_hopSize;
    int m_frameSize;
    BTrack m_btrack;
    
    std::atomic<bool> m_beatDetected;
    std::atomic<double> m_bpm;
    
    std::vector<double> m_audioFrame;  // Nur float→double Buffer, Größe = hopSize
};
