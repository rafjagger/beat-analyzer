/**
 * BeatAnalyzerApp — Audio + Beat Verarbeitung
 *
 * processAudio():           JACK RT-Callback → nur Ringbuffer-Copy
 * processBeatThread():      BTrack + TAP-Queue + Synthclock + OSC
 * sendBeatClockForChannel(): /beat OSC senden
 * sendVuMeterOsc():         /vu OSC Bundle senden
 */

#include "app/beat_analyzer_app.h"

#include <cstdio>
#include <algorithm>

using namespace BeatAnalyzer::Audio;
using namespace BeatAnalyzer::Util;

namespace BeatAnalyzer {

// ============================================================================
// processAudio() — JACK RT Callback
// ============================================================================
// Läuft im JACK-Realtime-Thread. Hier darf NICHTS Teures passieren:
// Kein malloc, kein mutex, kein I/O, keine FFT.
// VU: direkte float-writes (lock-free)
// BPM: Audio → SPSC Ringbuffer → BTrack-Thread

void BeatAnalyzerApp::processAudio(const std::vector<const CSAMPLE*>& bpmBuffers,
                                    const std::vector<const CSAMPLE*>& vuBuffers,
                                    int frameCount) {
    
    // VU Kanäle verarbeiten - KEIN LOCK (lock-free, nur float writes)
    for (int ch = 0; ch < m_numVuChannels && ch < static_cast<int>(vuBuffers.size()); ++ch) {
        const CSAMPLE* buffer = vuBuffers[ch];
        m_vuMeters[ch]->processMono(buffer, frameCount);
    }
    
    // BPM Kanäle: Audio nur in Ringbuffer kopieren (billig: ~512 bytes memcpy)
    // BTrack läuft NICHT im JACK-Callback — zu teuer (FFT + Transzendente)!
    for (int ch = 0; ch < m_numBpmChannels && ch < static_cast<int>(bpmBuffers.size()); ++ch) {
        auto& ring = m_audioRing[ch];
        int w = ring.wpos.load(std::memory_order_relaxed);
        int next = (w + 1) & AUDIO_RING_MASK;
        if (next != ring.rpos.load(std::memory_order_acquire)) {
            auto& slot = ring.slots[w];
            std::memcpy(slot.data, bpmBuffers[ch], frameCount * sizeof(float));
            slot.frameCount = frameCount;
            slot.endFrame = m_frameCount.load(std::memory_order_relaxed) + frameCount;
            ring.wpos.store(next, std::memory_order_release);
        }
        // Bei Ringbuffer-Overflow: Frame droppen (besser als XRun)
    }
    
    m_frameCount.fetch_add(frameCount, std::memory_order_relaxed);
}

// ============================================================================
// processBeatThread() — BTrack + Synthclock + TAP + OSC
// ============================================================================
// BTrack läuft HIER, nicht im JACK-Callback!
// JACK kopiert Audio in lock-free Ringbuffer → dieser Thread pollt und verarbeitet.

void BeatAnalyzerApp::processBeatThread() {
    const auto interval = std::chrono::microseconds(500);  // 2kHz polling
    
    auto nextWakeTime = std::chrono::steady_clock::now();
    
    while (g_running) {
        // ========================================
        // BTrack: Audio aus Ringbuffer verarbeiten (ALLE Modi)
        // BTrack muss IMMER laufen, damit BPM-Tracking bereit ist
        // wenn man in Modus 1 wechselt.
        // ========================================
        for (int ch = 0; ch < m_numBpmChannels; ++ch) {
            auto& ring = m_audioRing[ch];
            // Alle verfügbaren Frames verarbeiten
            while (true) {
                int r = ring.rpos.load(std::memory_order_relaxed);
                if (r == ring.wpos.load(std::memory_order_acquire)) break;
                
                auto& slot = ring.slots[r];
                bool beat = m_btrackDetectors[ch]->processSamples(slot.data, slot.frameCount);
                ring.rpos.store((r + 1) & AUDIO_RING_MASK, std::memory_order_release);
                
                // BTracks Beat mit seiner Position im Audio, nicht mit dem
                // Zeitpunkt, zu dem dieser Thread ihn abholt. Gemessen
                // 2026-09-17: diese Beats liegen auf ±3 ms, seine Tempo-Zahl
                // 1-4 BPM zu niedrig. Die Clock nimmt deshalb die Beats.
                if (beat) {
                    m_beatClocks[ch].trackerBeat(slot.endFrame);
                    if (m_debugBtrackConsole && ch == 0) {
                        printf("BTRACK_BEAT: frame=%lld bpm_btrack=%.1f bpm_clock=%.3f\n",
                               static_cast<long long>(slot.endFrame),
                               m_btrackDetectors[ch]->getBpm(), m_beatClocks[ch].bpm());
                        fflush(stdout);
                    }
                }
                m_btrackBpmValue[ch].store(m_btrackDetectors[ch]->getBpm(), std::memory_order_release);
            }
        }
        
        if (m_clockMode.load() != 1) {
            // Modus 0 (a3motion) und 2 (pioneer): externe Quelle,
            // Synthclock pausiert, /beat wird im Receiver-Callback gesendet
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            nextWakeTime = std::chrono::steady_clock::now();
            continue;
        }
        
        int64_t currentFrame = m_frameCount.load(std::memory_order_relaxed);
        
        // ========================================
        // TAP-Queue verarbeiten
        // ========================================
        while (true) {
            int r = m_tapQueueRead.load(std::memory_order_relaxed);
            if (r == m_tapQueueWrite.load(std::memory_order_acquire)) break;
            TapEvent tap = m_tapQueue[r];
            m_tapQueueRead.store((r + 1) & (TAP_QUEUE_SIZE - 1), std::memory_order_release);
            int tapBeat = tap.beat;
            {
                double tapDeltaMs = m_hasPrevTap 
                    ? std::chrono::duration<double, std::milli>(tap.timestamp - m_lastTapTime).count() 
                    : 0.0;
                
                bool isFirstTap = !m_hasPrevTap || tapDeltaMs > 2000.0;
                
                if (isFirstTap) {
                    m_tapIntervalCount = 0;
                    m_tapIntervalIndex = 0;
                    m_tapBpm = 0.0;
                    
                    // Erster TAP = Downbeat/1 definieren
                    for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                        m_bpmTrackStates[ch].beatNumber = 1;
                        m_beatClocks[ch].reset(m_beatClocks[ch].bpm(), currentFrame);
                    }
                    sendBeatClockForChannel(0, false);
                    
                    printf("TAP 1 | (erster Tap — Downbeat/1 definiert)\n");
                    fflush(stdout);
                } else {
                    m_tapIntervals[m_tapIntervalIndex % 8] = tapDeltaMs;
                    m_tapIntervalIndex++;
                    if (m_tapIntervalCount < 8) m_tapIntervalCount++;
                    
                    if (m_tapIntervalCount >= 2) {
                        double sorted[16];
                        int count = std::min(m_tapIntervalCount, 8);
                        for (int k = 0; k < count; ++k)
                            sorted[k] = m_tapIntervals[k];
                        std::sort(sorted, sorted + count);
                        double medianInterval = sorted[count / 2];
                        m_tapBpm = 60000.0 / medianInterval;
                    } else {
                        m_tapBpm = 60000.0 / tapDeltaMs;
                    }
                    
                    if (m_tapBpm >= m_bpmMin && m_tapBpm <= m_bpmMax) {
                        // BTrack Tempo hard-locken
                        for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                            m_btrackDetectors[ch]->fixTempo(m_tapBpm);
                            m_bpmTrackStates[ch].currentBpm = m_tapBpm;
                        }
                        
                        // TAP = Tempo und Phase-Reset auf 1
                        if (m_tapIntervalCount >= 2) {
                            for (int ch = 0; ch < m_numBpmChannels; ++ch) {
                                m_bpmTrackStates[ch].beatNumber = 1;
                                m_beatClocks[ch].reset(m_tapBpm, currentFrame);
                            }
                        }
                        
                        printf("TAP %d | delta %7.1fms | tapBPM %5.1f → fixTempo | taps=%d\n",
                               tapBeat, tapDeltaMs, m_tapBpm, m_tapIntervalCount);
                    } else {
                        printf("TAP %d | delta %7.1fms | tapBPM %5.1f (außerhalb %.0f-%.0f)\n",
                               tapBeat, tapDeltaMs, m_tapBpm, m_bpmMin, m_bpmMax);
                    }
                    fflush(stdout);
                }
                m_lastTapTime = tap.timestamp;
                m_hasPrevTap = true;
            }
        } // while (tap queue)
        
        // ========================================
        // BEATCLOCK
        // ========================================
        // Tempo und Phase aus BTracks Beats (BeatClockFollower). Die alte
        // Synthclock nahm nur BTracks Tempo-Zahl und zählte frei weiter —
        // bei einem 120er Klick mit 117,454 BPM, ein Beat Versatz alle 23 s.
        // TAP setzt Tempo, Phase und Beat 1 (siehe oben).
        for (int ch = 0; ch < m_numBpmChannels; ++ch) {
            auto& clock = m_beatClocks[ch];
            // Der Zustand bleibt die eine Wahrheit über die Schlagnummer:
            // Modus 0/2 schreiben sie aus empfangenen Beats.
            clock.setBeatNumber(m_bpmTrackStates[ch].beatNumber);
            if (clock.advance(currentFrame)) {
                m_bpmTrackStates[ch].beatNumber = clock.beatNumber();
                m_bpmTrackStates[ch].currentBpm = clock.bpm();
                sendBeatClockForChannel(ch, false);
            }
        }
        
        std::this_thread::sleep_until(nextWakeTime += interval);
    }
}

// ============================================================================
// sendBeatClockForChannel()
// ============================================================================

void BeatAnalyzerApp::sendBeatClockForChannel(int ch, bool isRealBeat) {
    if (!m_enableBeatclock) return;
    if (!m_oscSender || !m_oscSender->isConnected()) return;
    
    // Nur bei Modus 1 (intern) senden
    // Bei Modus 0/2 wird /beat im Receiver-Callback gesendet
    if (m_clockMode.load() != 1) return;
    
    double effectiveBpm = m_bpmTrackStates[ch].currentBpm;
    
    // /beat iif  beat(1-4), bar, bpm
    OSC::BeatClockMessage msg;
    msg.track_id = ch;
    msg.beat_number = m_bpmTrackStates[ch].beatNumber;
    msg.bar_number = m_bpmTrackStates[ch].barNumber;
    msg.bpm = effectiveBpm;
    
    m_oscSender->sendBeatClock(msg);
    
    // Timing-Analyse: Jeden Beat mit Timestamp + Delta ausgeben
    if (m_debugBeatConsole) {
        static auto lastBeatTime = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        double deltaMs = std::chrono::duration<double, std::milli>(now - lastBeatTime).count();
        lastBeatTime = now;
        
        double expectedMs = (msg.bpm > 0) ? (60000.0 / msg.bpm) : 0.0;
        double jitterMs = (expectedMs > 0) ? (deltaMs - expectedMs) : 0.0;
        
        printf("BEAT %d | BPM %5.1f | delta %7.1fms | expected %7.1fms | jitter %+.1fms | %s\n",
               msg.beat_number, msg.bpm, deltaMs, expectedMs, jitterMs,
               isRealBeat ? "REAL" : "SYNTH");
        fflush(stdout);
    }
}

// ============================================================================
// sendVuMeterOsc()
// ============================================================================

void BeatAnalyzerApp::sendVuMeterOsc() {
    if (!m_enableVu) return;
    if (!m_oscSender || !m_oscSender->isConnected()) return;
    
    // Alle VU-Werte sammeln und als ein OSC Bundle senden
    // 1 UDP-Paket statt 12 — atomar, effizient
    float peaks[MAX_VU_CHANNELS];
    float rms[MAX_VU_CHANNELS];
    
    for (int ch = 0; ch < m_numVuChannels; ++ch) {
        peaks[ch] = m_vuMeters[ch]->getPeakLinear();
        rms[ch] = m_vuMeters[ch]->getRmsLinear();
    }
    
    m_oscSender->sendVuBundle(m_vuOscPaths.data(), peaks, rms, m_numVuChannels);
    
    // DEBUG: /vu/3 auf Konsole (nur wenn aktiviert)
    if (m_debugVuConsole && m_numVuChannels > 3) {
        static auto lastVuTime = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        double deltaMs = std::chrono::duration<double, std::milli>(now - lastVuTime).count();
        lastVuTime = now;
        printf("/vu/3 | peak %.3f | rms %.3f | delta %6.1fms\n", peaks[3], rms[3], deltaMs);
        fflush(stdout);
    }
}

} // namespace BeatAnalyzer
