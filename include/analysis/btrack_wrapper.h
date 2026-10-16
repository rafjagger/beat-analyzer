#pragma once

#include "BTrack.h"
#include <atomic>
#include <optional>
#include <vector>

/**
 * BTrackWrapper - Minimaler Wrapper um BTrack
 *
 * BTrack runs on a fixed hop (frameSize = hopSize * 2, as in the Max
 * external). JACK's period is not fixed: PipeWire delivers whatever quantum
 * the graph runs at and can change it while running. The wrapper therefore
 * collects any block size into whole hops and hands BTrack one hop at a time.
 *
 * Until 2026-10-16 every block went to BTrack as one hop, sized at start-up:
 * a quantum of 256 against a hop of 128 wrote past the frame buffer and
 * crashed the analyzer on a3nuc1.
 *
 * Referenz: external/BTrack/plugins/max-external/btrack~/btrack~.cpp
 */
class BTrackWrapper {
public:
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
     * One JACK block of any size.
     *
     * Returns, if a beat fell in this block, how many samples before the
     * block's end the hop that carried it ended; nothing otherwise. The
     * caller's beat frame is then blockEndFrame - that number.
     */
    std::optional<int> processSamples(const float* samples, int numSamples) {
        std::optional<int> samplesAfterBeat;
        for (int i = 0; i < numSamples; ++i) {
            m_audioFrame[static_cast<size_t>(m_filled++)] = static_cast<double>(samples[i]);
            if (m_filled < m_hopSize) continue;
            m_filled = 0;
            if (processHop())
                samplesAfterBeat = numSamples - 1 - i;
        }
        return samplesAfterBeat;
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

    /**
     * Die gemessene Beat-Periode in Samples an BTrack geben, 0 = keine.
     *
     * BTrack sagt den nächsten Beat mit der Periode seiner 2-BPM-Tempostufe
     * voraus. Gemessen 2026-09-17: dadurch liegt die Latenz je nach Tempo
     * sägezahnförmig zwischen 4 und 17 ms und springt an jeder Stufenkante
     * um 10 ms. Mit der aus den Beat-Abständen gemessenen Periode liegt sie
     * bei allen Tempi zwischen -2 und +3 ms.
     */
    void setBeatPeriodSamples(double samples) {
        m_btrack.setMeasuredBeatPeriod(samples);
    }

    int getHopSize() const { return m_hopSize; }
    int getFrameSize() const { return m_frameSize; }

private:
    bool processHop() {
        m_btrack.processAudioFrame(m_audioFrame.data());
        bool const beatNow = m_btrack.beatDueInCurrentFrame();
        if (beatNow)
            m_beatDetected.store(true, std::memory_order_release);
        m_bpm.store(m_btrack.getCurrentTempoEstimate(), std::memory_order_release);
        return beatNow;
    }

    int m_hopSize;
    int m_frameSize;
    BTrack m_btrack;
    
    std::atomic<bool> m_beatDetected;
    std::atomic<double> m_bpm;
    
    std::vector<double> m_audioFrame;  // one hop, filled across blocks
    int m_filled = 0;                  // samples of the current hop so far
};
