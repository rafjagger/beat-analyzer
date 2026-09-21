#include "analysis/beat_clock_follower.h"

#include <algorithm>
#include <cmath>

namespace BeatAnalyzer {
namespace Analysis {

BeatClockFollower::BeatClockFollower(double sampleRate, double minBpm, double maxBpm)
    : m_sampleRate(sampleRate), m_minBpm(minBpm), m_maxBpm(maxBpm) {}

void BeatClockFollower::trackerBeat(int64_t frame) {
    if (m_lastTrackerBeat >= 0 && frame > m_lastTrackerBeat) {
        auto const raw = static_cast<double>(frame - m_lastTrackerBeat);
        keepInterval(raw);
    }
    m_lastTrackerBeat = frame;

    auto const period = clockPeriod();
    if (period <= 0.0) return;

    // Die Oktave, gegen die der naechste Abstand gemessen wird.
    m_lockedPeriod = period;

    if (!m_running) {
        // Der erste Beat, zu dem ein Tempo bekannt ist, setzt die Phase.
        m_running = true;
        m_nextBeat = static_cast<double>(frame) + period;
        return;
    }

    // Wie weit die Clock neben dem Tracker-Beat liegt, modulo der
    // Tracker-Periode: bei gefalteter Oktave liegt jeder zweite Tracker-Beat
    // zwischen zwei Clock-Beats und ist trotzdem richtig.
    auto const tracker = trackerPeriod();
    auto const error = std::remainder(static_cast<double>(frame) - m_nextBeat, tracker);

    if (std::fabs(error) <= lockWindow * tracker) {
        m_hasPendingJump = false;
        m_nextBeat += gain * error;
    } else if (m_hasPendingJump
               && std::fabs(std::remainder(error - m_pendingJump, tracker))
                      <= lockWindow * tracker) {
        // Zweimal hintereinander gleich weit daneben: das ist die neue
        // Phase, kein Ausreißer.
        m_hasPendingJump = false;
        m_nextBeat += error;
    } else {
        // Einmal weit daneben kann ein verirrter Beat sein. Merken, noch
        // nicht springen.
        m_hasPendingJump = true;
        m_pendingJump = error;
    }

    // Nie einen Beat nachholen, der schon gesendet ist.
    while (m_nextBeat <= static_cast<double>(m_lastBeatFrame) + 0.5 * period)
        m_nextBeat += period;
}

void BeatClockFollower::reset(double bpm, int64_t frame) {
    if (bpm <= 0.0) return;
    m_fallbackPeriod = m_sampleRate * 60.0 / bpm;
    m_intervalCount = 0;
    m_intervalIndex = 0;
    m_lastTrackerBeat = -1;
    m_running = true;
    m_hasPendingJump = false;
    m_lastBeatFrame = frame;
    m_nextBeat = static_cast<double>(frame) + m_fallbackPeriod;
}

bool BeatClockFollower::advance(int64_t frame) {
    if (!m_running) return false;
    auto const period = clockPeriod();
    if (period <= 0.0 || static_cast<double>(frame) < m_nextBeat) return false;

    m_lastBeatFrame = static_cast<int64_t>(std::llround(m_nextBeat));
    m_nextBeat += period;
    // Nach einer langen Pause keine Salve nachholen: ein Beat, dann weiter.
    while (m_nextBeat <= static_cast<double>(frame))
        m_nextBeat += period;
    m_beatNumber = (m_beatNumber % 4) + 1;
    return true;
}

double BeatClockFollower::bpm() const {
    auto const period = clockPeriod();
    return period > 0.0 ? m_sampleRate * 60.0 / period : 0.0;
}

/** Einen Tracker-Abstand aufnehmen, in der Oktave, in der die Clock laeuft.
 *
 *  Hier sitzt die Oktav-Sperre, und sie sitzt auf der *Zeit*, nicht auf einem
 *  Tempobereich. Ein Bereich kann nur sagen, was erlaubt ist -- nicht, was
 *  gemeint war; und er muesste genau eine Oktave breit sein, um ueberhaupt
 *  etwas zu entscheiden, was Tempi unterhalb davon ausschliesst. Der
 *  Maintainer will beides: kein Springen zwischen 70 und 140, und Tempi unter
 *  70 sollen trotzdem erkannt werden.
 *
 *  Also wird gefaltet, sobald die Clock eingerastet ist: ein Abstand nahe dem
 *  Doppelten ist derselbe Takt, halb gezaehlt -- BTrack laesst in einem
 *  Breakdown jeden zweiten Beat aus. Gefaltet *bevor* er in die Historie
 *  geht, damit der Median gar nicht erst verschmutzt: sonst dauert es sechzehn
 *  Beats, bis er kippt, und noch einmal so lange zurueck.
 *
 *  Bleibt die andere Oktave bestehen, ist es ein Trackwechsel. Dann wird die
 *  Historie verworfen und auf dem rohen Abstand neu aufgebaut, damit das neue
 *  Tempo sofort und nicht gemittelt gilt.
 */
void BeatClockFollower::keepInterval(double raw) {
    auto interval = raw;
    auto folded = false;

    if (m_lockedPeriod > 0.0) {
        while (interval > 1.5 * m_lockedPeriod) { interval *= 0.5; folded = true; }
        while (interval < 0.67 * m_lockedPeriod) { interval *= 2.0; folded = true; }
    }

    m_octaveStreak = folded ? m_octaveStreak + 1 : 0;

    if (folded && m_octaveStreak >= octaveChangeBeats) {
        m_intervalCount = 0;
        m_intervalIndex = 0;
        m_octaveStreak = 0;
        m_lockedPeriod = 0.0;
        interval = raw;
    }

    m_intervals[m_intervalIndex] = interval;
    m_intervalIndex = (m_intervalIndex + 1) % intervalHistory;
    if (m_intervalCount < intervalHistory) ++m_intervalCount;
}

double BeatClockFollower::trackerPeriod() const {
    if (m_intervalCount == 0) return m_fallbackPeriod;
    double sorted[intervalHistory];
    std::copy(m_intervals, m_intervals + m_intervalCount, sorted);
    std::sort(sorted, sorted + m_intervalCount);
    auto const median = sorted[m_intervalCount / 2];

    // Der Median sortiert aus, der Mittelwert misst: ein Median aus
    // verrauschten Abständen ist nur so fein wie ein einzelner Abstand, der
    // Mittelwert der passenden wird mit jedem Beat genauer. Passend heißt
    // innerhalb von inlierTolerance — enger als ein Tempowechsel, den man
    // hören würde, damit alte und neue Abstände nicht gemittelt werden.
    double sum = 0.0;
    int count = 0;
    for (int i = 0; i < m_intervalCount; ++i) {
        if (std::fabs(sorted[i] - median) <= inlierTolerance * median) {
            sum += sorted[i];
            ++count;
        }
    }
    return count > 0 ? sum / count : median;
}

double BeatClockFollower::clockPeriod() const {
    auto period = trackerPeriod();
    if (period <= 0.0) return 0.0;
    auto const maxPeriod = m_sampleRate * 60.0 / m_minBpm;
    auto const minPeriod = m_sampleRate * 60.0 / m_maxBpm;
    while (period < minPeriod) period *= 2.0;
    while (period > maxPeriod) period *= 0.5;
    return period;
}

} // namespace Analysis
} // namespace BeatAnalyzer
