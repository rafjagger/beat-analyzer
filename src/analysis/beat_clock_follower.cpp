#include "analysis/beat_clock_follower.h"

#include <algorithm>
#include <cmath>

namespace BeatAnalyzer {
namespace Analysis {

BeatClockFollower::BeatClockFollower(double sampleRate, double minBpm, double maxBpm)
    : m_sampleRate(sampleRate), m_maxBpm(maxBpm) {
    // Genau eine Oktave, verankert am schnellen Ende.
    //
    // clockPeriod() faltet eine Periode in [m_minBpm, m_maxBpm] -- aber nur,
    // wenn dort genau eine Lesart Platz hat. Ist der Bereich breiter als eine
    // Oktave, liegen 70 und 140 beide darin, beide sind "gueltig", und die
    // Sperre sperrt nichts: BTrack darf zwischen den Oktaven wechseln und die
    // Periode folgt brav mit. Genau das war "tempo 70 und 140 springt",
    // gemeldet am 2026-09-19, mit dem damaligen Bereich 60-140.
    //
    // Am schnellen Ende verankert und nicht am langsamen, weil in Tanzmusik
    // die gezaehlte Zahl die schnellere ist: ein 70er Feel wird 140 gezaehlt.
    // Andersherum verankert haette dieselbe Konfiguration jeden 140er Track
    // auf 70 gezogen.
    m_minBpm = (minBpm > 0.0 && maxBpm > 2.0 * minBpm) ? maxBpm * 0.5 : minBpm;
}

void BeatClockFollower::trackerBeat(int64_t frame) {
    if (m_lastTrackerBeat >= 0 && frame > m_lastTrackerBeat) {
        m_intervals[m_intervalIndex] = static_cast<double>(frame - m_lastTrackerBeat);
        m_intervalIndex = (m_intervalIndex + 1) % intervalHistory;
        if (m_intervalCount < intervalHistory) ++m_intervalCount;
    }
    m_lastTrackerBeat = frame;

    auto const period = clockPeriod();
    if (period <= 0.0) return;

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
