/**
 * BTrack mit gemessener Beat-Periode — Latenz ohne Sägezahn.
 *
 * Derselbe Rauschklick wie smoke-test/scripts/beat-click.c, samplegenau
 * durch BTrackWrapper (Hop 256, 44100 Hz) und BeatClockFollower, so
 * verdrahtet wie in beat_processing.cpp.
 *
 * Gemessen 2026-09-17: ohne die Periode liegt die Latenz der Clock hinter dem
 * Klick je nach Tempo zwischen 4 und 17 ms (118: +3,8, 120,5: +14,6,
 * 121: +4,9, 132: +17,2) und springt an BTracks 2-BPM-Stufenkanten.
 */

#include "analysis/beat_clock_follower.h"
#include "analysis/btrack_wrapper.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
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
static const int hop = 256;

struct Run {
    std::vector<double> clicks;
    std::vector<double> clockBeats;
    double bpm = 0.0;
};

/** Klick bei `bpm`, ab `changeAt` Sekunden bei `bpmAfter`. */
static Run play(double bpm, double seconds, double bpmAfter = 0.0, double changeAt = 1e9) {
    Run run;
    BTrackWrapper tracker(hop, hop * 2);
    BeatClockFollower clock(rate);
    std::vector<float> buf(hop);
    uint32_t noise = 22222;
    int const clickLen = static_cast<int>(rate * 0.03);
    double nextClick = rate * 0.5;
    long playing = -1;

    for (long pos = 0; pos + hop <= static_cast<long>(seconds * rate); pos += hop) {
        for (int i = 0; i < hop; ++i) {
            long const n = pos + i;
            if (static_cast<double>(n) >= nextClick) {
                run.clicks.push_back(static_cast<double>(n));
                double const now = n / rate < changeAt ? bpm : bpmAfter;
                nextClick += rate * 60.0 / now;
                playing = 0;
            }
            float v = 0.0f;
            if (playing >= 0 && playing < clickLen) {
                noise = noise * 1664525u + 1013904223u;
                v = 0.9f * static_cast<float>(std::exp(-6.0 * playing / clickLen))
                    * (static_cast<int32_t>(noise) / 2147483648.0f);
                ++playing;
            }
            buf[static_cast<size_t>(i)] = v;
        }
        long const end = pos + hop;
        if (tracker.processSamples(buf.data(), hop)) {
            clock.trackerBeat(end);
            tracker.setBeatPeriodSamples(clock.trackerPeriodSamples());
        }
        if (clock.advance(end)) run.clockBeats.push_back(static_cast<double>(clock.lastBeatFrame()));
    }
    run.bpm = clock.bpm();
    return run;
}

static double offsetMs(Run const& run, double beat) {
    double best = 1e18;
    for (double c : run.clicks)
        if (std::fabs(beat - c) < std::fabs(best)) best = beat - c;
    return best / rate * 1000.0;
}

static void latency(double bpm) {
    auto const run = play(bpm, 60.0);
    double sum = 0.0;
    int n = 0;
    for (double b : run.clockBeats)
        if (b > 20 * rate) { sum += offsetMs(run, b); ++n; }
    double const mean = n ? sum / n : 1e9;
    std::printf("  %6.1f BPM: latency %+.1f ms, clock %.3f BPM\n", bpm, mean, run.bpm);
    CHECK(n >= 55, "only %d beats at %.1f BPM", n, bpm);
    CHECK(std::fabs(mean) < 5.0, "latency %+.1f ms at %.1f BPM", mean, bpm);
    CHECK(std::fabs(run.bpm - bpm) < 0.1, "clock %.3f BPM at %.1f", run.bpm, bpm);
}

static void theLatencyNoLongerDependsOnTheTempo() {
    std::printf("Testing latency across tempi (the old sawtooth)...\n");
    for (double bpm : {90.0, 118.0, 120.0, 120.5, 121.0, 124.0, 132.0})
        latency(bpm);
}

static void aTempoChangeStillComesThrough(double to) {
    // Die Periode, die BTrack bekommt, stammt aus seinen eigenen Beats. Das
    // darf ihn nicht auf dem alten Tempo festhalten, wenn die Musik wechselt.
    auto const run = play(120.0, 70.0, to, 30.0);
    double sum = 0.0;
    int n = 0;
    for (double b : run.clockBeats)
        if (b > 55 * rate) { sum += offsetMs(run, b); ++n; }
    double const mean = n ? sum / n : 1e9;
    std::printf("  120 -> %.1f BPM: clock %.3f BPM, latency %+.1f ms\n", to, run.bpm, mean);
    CHECK(std::fabs(run.bpm - to) < 0.15, "clock %.3f BPM after a change to %.1f", run.bpm, to);
    CHECK(std::fabs(mean) < 6.0, "latency %+.1f ms after a change to %.1f", mean, to);
}

static void theWiringIsInTheApp() {
    // Ohne Aufrufer in der App ist der Patch wirkungslos und jeder Test hier
    // trotzdem grün.
    std::ifstream in(BEAT_ANALYZER_SOURCE_DIR "/src/app/beat_processing.cpp");
    std::stringstream text;
    text << in.rdbuf();
    CHECK(text.str().find("setBeatPeriodSamples(") != std::string::npos,
          "beat_processing.cpp never hands BTrack the measured period");
}

int main() {
    theLatencyNoLongerDependsOnTheTempo();
    std::printf("Testing tempo changes...\n");
    aTempoChangeStillComesThrough(126.0);
    aTempoChangeStillComesThrough(122.5);
    aTempoChangeStillComesThrough(115.0);
    theWiringIsInTheApp();
    if (failures) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("  ✓ all BTrack period tests passed\n");
    return 0;
}
