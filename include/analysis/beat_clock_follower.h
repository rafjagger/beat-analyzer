#pragma once

#include <cstdint>

namespace BeatAnalyzer {
namespace Analysis {

/**
 * Die Beatclock, die /beat sendet — Tempo und Phase aus BTracks Beats.
 *
 * Vorher nahm die Synthclock von BTrack nur die Tempo-Zahl und zählte selbst
 * weiter. Gemessen am 2026-09-17 mit einem Klick bekannten Tempos: die Zahl
 * liegt 1–4 BPM zu niedrig (BTrack schätzt in 2-BPM-Stufen und rundet die
 * Periode auf ganze Hops), BTracks Beats dagegen liegen auf ±3 ms und ihr
 * Abstand trifft das Tempo auf ±0,01 BPM. Bei einem 120er Klick lief die
 * Clock mit 117,454 BPM frei und rutschte alle 23 s einen ganzen Beat.
 *
 * - Tempo: Mittelwert der letzten Beat-Abstände, die nahe an ihrem Median
 *   liegen. Ein verirrter Beat fällt heraus, ein echter Tempowechsel setzt
 *   sich durch, sobald er die Hälfte der Historie stellt.
 * - Phase: jeder Tracker-Beat zieht die Clock innerhalb eines Viertelbeats
 *   um einen Anteil heran. Liegt er weiter daneben, springt die Clock erst,
 *   wenn der nächste Beat das bestätigt — ein einzelner verirrter Beat ist
 *   kein neuer Takt.
 * - Oktave: außerhalb [minBpm, maxBpm] wird die Clock-Periode verdoppelt
 *   oder halbiert — wie bisher 60–140. Die Phase wird dann modulo der
 *   Tracker-Periode verglichen, sonst sähe jeder zweite Beat falsch aus.
 * - Ohne Tracker-Beats läuft die Clock mit dem letzten Tempo weiter.
 *
 * Alles in Audio-Frames, kein Thread, keine Uhr: testbar mit synthetischen
 * Beats.
 */
class BeatClockFollower {
public:
    explicit BeatClockFollower(double sampleRate, double minBpm = 60.0,
                               double maxBpm = 140.0);

    /** BTrack hat einen Beat bei `frame` gemeldet. */
    void trackerBeat(int64_t frame);

    /** Tempo und Phase setzen: bei `frame` war ein Beat (TAP). Der nächste
     *  kommt eine Periode später — der bei `frame` gilt als gesendet. */
    void reset(double bpm, int64_t frame);

    /** Bis `frame` vorrücken. true, wenn dabei ein Beat fällig war. */
    bool advance(int64_t frame);

    double bpm() const;
    int64_t lastBeatFrame() const { return m_lastBeatFrame; }
    /** 1–4, wie /beat es zählt. */
    int beatNumber() const { return m_beatNumber; }
    void setBeatNumber(int beat) { m_beatNumber = beat; }

    static constexpr int intervalHistory = 32;
    static constexpr double inlierTolerance = 0.03;
    static constexpr double lockWindow = 0.25;
    static constexpr double gain = 0.3;

private:
    double trackerPeriod() const;
    double clockPeriod() const;

    double m_sampleRate;
    double m_minBpm;
    double m_maxBpm;

    int64_t m_lastTrackerBeat = -1;
    double m_intervals[intervalHistory] = {};
    int m_intervalCount = 0;
    int m_intervalIndex = 0;
    double m_fallbackPeriod = 0.0;

    bool m_hasPendingJump = false;
    double m_pendingJump = 0.0;

    bool m_running = false;
    double m_nextBeat = 0.0;
    int64_t m_lastBeatFrame = 0;
    int m_beatNumber = 1;
};

} // namespace Analysis
} // namespace BeatAnalyzer
