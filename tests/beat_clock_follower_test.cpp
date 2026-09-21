/**
 * BeatClockFollower — die Clock folgt BTracks Beats, nicht seiner Tempo-Zahl.
 *
 * Gemessen am 2026-09-17 mit einem Klick bekannten Tempos (siehe
 * a3-system/issues/a3-motion-ui-haelt-den-takt-nicht.md): BTracks Beats liegen
 * auf ±3 ms und ihr Abstand trifft das Tempo auf ±0,01 BPM, seine gemeldete
 * Tempo-Zahl liegt 1–4 BPM zu niedrig. Die Synthclock nahm die Zahl und
 * ignorierte die Beats — sie lief bei einem 120er Klick mit 117,454 BPM frei.
 *
 * Eigenes CHECK statt assert: assert verschwindet im Release-Build.
 */

#include "analysis/beat_clock_follower.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

using BeatAnalyzer::Analysis::BeatClockFollower;

static int failures = 0;
#define CHECK(cond, ...)                                                   \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::printf("  FAIL %s:%d: %s — ", __FILE__, __LINE__, #cond); \
            std::printf(__VA_ARGS__);                                      \
            std::printf("\n");                                             \
            ++failures;                                                    \
        }                                                                  \
    } while (0)

static const double rate = 44100.0;

/** Tracker-Beats bei `bpm`, mit ±`jitter` Frames Rauschen, ab `start`. */
static std::vector<int64_t> trackerBeats(double bpm, double seconds, int jitter,
                                         double start = 22050.0) {
    std::vector<int64_t> out;
    uint32_t noise = 12345;
    double const period = rate * 60.0 / bpm;
    for (double t = start; t < seconds * rate; t += period) {
        noise = noise * 1664525u + 1013904223u;
        int const j = jitter > 0 ? static_cast<int>(noise % (2 * jitter + 1)) - jitter : 0;
        out.push_back(static_cast<int64_t>(std::llround(t)) + j);
    }
    return out;
}

/** Spielt Tracker-Beats in 256er Schritten ein; gibt die Clock-Beats zurück. */
static std::vector<int64_t> run(BeatClockFollower& clock,
                                std::vector<int64_t> const& beats,
                                double seconds, int hop = 256) {
    std::vector<int64_t> emitted;
    size_t next = 0;
    for (int64_t frame = 0; frame < seconds * rate; frame += hop) {
        int64_t const end = frame + hop;
        while (next < beats.size() && beats[next] < end) {
            clock.trackerBeat(beats[next]);
            ++next;
        }
        if (clock.advance(end))
            emitted.push_back(clock.lastBeatFrame());
    }
    return emitted;
}

static double nearestOffsetMs(std::vector<int64_t> const& truth, int64_t frame) {
    double best = 1e18;
    for (auto t : truth)
        if (std::fabs(double(frame - t)) < std::fabs(best)) best = double(frame - t);
    return best / rate * 1000.0;
}

static void theTempoComesFromTheBeatsNotFromAnEstimate() {
    std::printf("Testing tempo from beat intervals...\n");
    BeatClockFollower clock(rate);
    auto const truth = trackerBeats(120.0, 60.0, 0);
    auto const beats = trackerBeats(120.0, 60.0, 256);
    run(clock, beats, 60.0);
    CHECK(std::fabs(clock.bpm() - 120.0) < 0.1, "bpm %.3f", clock.bpm());
}

static void theClockSitsOnTheBeats() {
    std::printf("Testing phase lock...\n");
    BeatClockFollower clock(rate);
    auto const truth = trackerBeats(120.0, 60.0, 0);
    auto const emitted = run(clock, trackerBeats(120.0, 60.0, 256), 60.0);
    int late = 0;
    double worst = 0;
    for (auto e : emitted) {
        if (e < 10 * rate) continue;
        double const off = nearestOffsetMs(truth, e);
        worst = std::max(worst, std::fabs(off));
        if (std::fabs(off) > 8.0) ++late;
    }
    CHECK(emitted.size() >= 110, "only %zu beats in a minute at 120", emitted.size());
    CHECK(late == 0, "%d beats more than 8 ms off, worst %.1f ms", late, worst);
}

static void itLocksOnWhenItStartsOnTheWrongPhase() {
    std::printf("Testing lock from the wrong phase...\n");
    BeatClockFollower clock(rate);
    // Die Clock kennt schon das Tempo, liegt aber einen halben Beat daneben.
    clock.reset(120.0, 0);
    auto const truth = trackerBeats(120.0, 30.0, 0, 11025.0 + 22050.0);
    auto const emitted = run(clock, truth, 30.0);
    int off = 0;
    for (auto e : emitted)
        if (e > 5 * rate && std::fabs(nearestOffsetMs(truth, e)) > 3.0) ++off;
    CHECK(emitted.size() >= 50, "only %zu beats in thirty seconds", emitted.size());
    CHECK(off == 0, "%d beats still off after five seconds", off);
}

static void aMissingStretchOfBeatsDoesNotStopTheClock() {
    std::printf("Testing a gap in the tracker's beats...\n");
    BeatClockFollower clock(rate);
    auto beats = trackerBeats(120.0, 40.0, 0);
    std::vector<int64_t> gapped;
    for (auto b : beats)
        if (b < 20 * rate || b > 26 * rate) gapped.push_back(b);
    auto const emitted = run(clock, gapped, 40.0);
    int inGap = 0;
    for (auto e : emitted)
        if (e > 20.5 * rate && e < 25.5 * rate) ++inGap;
    CHECK(inGap >= 9, "only %d beats in a five-second gap", inGap);
}

static void oneStrayBeatDoesNotMoveTheTempo() {
    std::printf("Testing a stray tracker beat...\n");
    BeatClockFollower clock(rate);
    auto beats = trackerBeats(120.0, 30.0, 0);
    // Kurz vor dem nächsten echten Beat: ein sofortiger Sprung würde hier einen
    // Beat 150 ms zu früh senden, bevor der echte ihn zurückholt.
    beats.push_back(static_cast<int64_t>(15.35 * rate));
    std::sort(beats.begin(), beats.end());
    auto const emitted = run(clock, beats, 30.0);
    CHECK(std::fabs(clock.bpm() - 120.0) < 0.1, "bpm %.3f", clock.bpm());
    int doubled = 0;
    for (size_t i = 1; i < emitted.size(); ++i)
        if (emitted[i] - emitted[i - 1] < 0.75 * 22050) ++doubled;
    CHECK(doubled == 0, "%d beats emitted too close together", doubled);
    // Und kein Phasensprung: ein einzelner Beat daneben ist ein Ausreißer,
    // kein neuer Takt. Erst ein zweiter, der ihn bestätigt, darf springen.
    // Die Referenz reicht über das Ende hinaus: der letzte Clock-Beat darf
    // genau auf 30 s fallen.
    auto const truth = trackerBeats(120.0, 31.0, 0);
    int off = 0;
    for (auto e : emitted)
        if (e > 5 * rate && std::fabs(nearestOffsetMs(truth, e)) > 3.0) ++off;
    CHECK(off == 0, "%d beats knocked off the beat by one stray", off);
}

static void aTempoChangeIsFollowed() {
    std::printf("Testing a tempo change...\n");
    BeatClockFollower clock(rate);
    auto beats = trackerBeats(120.0, 20.0, 0);
    auto later = trackerBeats(126.0, 40.0, 0, double(beats.back()) + rate * 60.0 / 126.0);
    beats.insert(beats.end(), later.begin(), later.end());
    run(clock, beats, 40.0);
    CHECK(std::fabs(clock.bpm() - 126.0) < 0.1, "bpm %.3f", clock.bpm());
}

static void doubleTimeIsFoldedIntoTheRange() {
    std::printf("Testing octave folding...\n");
    BeatClockFollower clock(rate, 60.0, 120.0);
    auto const beats = trackerBeats(150.0, 30.0, 0);
    auto const emitted = run(clock, beats, 30.0);
    CHECK(std::fabs(clock.bpm() - 75.0) < 0.1, "bpm %.3f", clock.bpm());
    int off = 0;
    for (auto e : emitted)
        if (e > 5 * rate && std::fabs(nearestOffsetMs(beats, e)) > 3.0) ++off;
    CHECK(emitted.size() >= 30, "only %zu beats in thirty seconds", emitted.size());
    CHECK(off == 0, "%d half-time beats not on a tracker beat", off);
}

// Gemeldet am 2026-09-19: "tempo 70 und 140 springt". Beides lag im Bereich
// 60-140, den die Clock mangels durchgereichter Konfiguration immer benutzte,
// also faltete sie nichts -- BTrack durfte zwischen den Oktaven wechseln und
// die Periode folgte brav mit.
static void halfTimeIsFoldedUpIntoTheRange() {
    std::printf("Testing half-time folding...\n");
    BeatClockFollower clock(rate, 80.0, 160.0);
    auto const beats = trackerBeats(70.0, 30.0, 0);
    run(clock, beats, 30.0);
    CHECK(std::fabs(clock.bpm() - 140.0) < 0.1, "bpm %.3f", clock.bpm());
}

// Und die Nachforderung, die den ersten Entwurf umgestossen hat: *"aber es
// sollen auch tempi unter 70 korrekt funktionieren und erkannt werden."* Ein
// fester Oktavbereich haette das ausgeschlossen -- 65 waere auf 130
// hochgefaltet worden. Der Bereich bleibt deshalb weit, und die Sperre sitzt
// auf der Zeit statt auf dem Bereich.
static void aSlowTempoIsNotPulledUp() {
    std::printf("Testing that a slow tempo stays slow...\n");
    BeatClockFollower clock(rate, 60.0, 160.0);
    auto const beats = trackerBeats(65.0, 40.0, 0);
    run(clock, beats, 40.0);
    CHECK(std::fabs(clock.bpm() - 65.0) < 0.2, "bpm %.3f", clock.bpm());
}

// Der Kern der Sache: BTrack faengt an, jeden zweiten Beat auszulassen. Die
// Abstaende verdoppeln sich, und ohne Sperre folgt die Periode brav mit -- das
// war "tempo 70 und 140 springt". Ein Abstand nahe dem Doppelten der
// eingerasteten Periode ist derselbe Takt, halb gezaehlt, und wird gefaltet.
static void halfTimeBeatsDoNotHalveTheTempo() {
    std::printf("Testing octave hysteresis...\n");
    BeatClockFollower clock(rate, 60.0, 160.0);

    auto beats = trackerBeats(140.0, 20.0, 0);
    // Ab Sekunde 20 nur noch jeder zweite Beat, zehn Sekunden lang -- etwa
    // zwoelf Beats, unter der Schwelle, ab der ein Trackwechsel angenommen
    // wird.
    auto const halved = trackerBeats(70.0, 30.0, 0, 20.0 * rate);
    beats.insert(beats.end(), halved.begin(), halved.end());

    run(clock, beats, 30.0);
    CHECK(std::fabs(clock.bpm() - 140.0) < 0.5, "bpm %.3f", clock.bpm());
}

// Aber nicht fuer immer: bleibt die andere Oktave bestehen, ist es ein
// Trackwechsel und kein Aussetzer. Die Sperre haelt das Tempo, sie friert es
// nicht ein.
static void aSustainedOctaveChangeIsFollowed() {
    std::printf("Testing that a lasting octave change is adopted...\n");
    BeatClockFollower clock(rate, 60.0, 160.0);

    auto beats = trackerBeats(140.0, 20.0, 0);
    auto const halved = trackerBeats(70.0, 120.0, 0, 20.0 * rate);
    beats.insert(beats.end(), halved.begin(), halved.end());

    run(clock, beats, 120.0);
    CHECK(std::fabs(clock.bpm() - 70.0) < 0.5, "bpm %.3f", clock.bpm());
}

// Und die Gegenprobe: was schon in der Oktave liegt, wird nicht angefasst.
static void aTempoInsideTheOctaveIsLeftAlone() {
    std::printf("Testing that the octave leaves the tempo alone...\n");
    BeatClockFollower clock(rate, 60.0, 160.0);
    auto const beats = trackerBeats(128.0, 30.0, 0);
    run(clock, beats, 30.0);
    CHECK(std::fabs(clock.bpm() - 128.0) < 0.1, "bpm %.3f", clock.bpm());
}

int main() {
    theTempoComesFromTheBeatsNotFromAnEstimate();
    theClockSitsOnTheBeats();
    itLocksOnWhenItStartsOnTheWrongPhase();
    aMissingStretchOfBeatsDoesNotStopTheClock();
    oneStrayBeatDoesNotMoveTheTempo();
    aTempoChangeIsFollowed();
    doubleTimeIsFoldedIntoTheRange();
    halfTimeIsFoldedUpIntoTheRange();
    aTempoInsideTheOctaveIsLeftAlone();
    aSlowTempoIsNotPulledUp();
    halfTimeBeatsDoNotHalveTheTempo();
    aSustainedOctaveChangeIsFollowed();
    if (failures) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("  ✓ all BeatClockFollower tests passed\n");
    return 0;
}
