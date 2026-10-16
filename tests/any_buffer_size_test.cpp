/**
 * BTrack at any JACK period size.
 *
 * PipeWire delivers whatever quantum the graph runs at, and it can change
 * while the analyzer runs. On a3nuc1 (2026-10-16) a forced quantum of 256
 * against a tracker built for 128 crashed the analyzer every few seconds.
 *
 * The same click track goes through a tracker with hop 128 in blocks of every
 * size JACK can deliver, regular and changing mid-run. The beats must land on
 * the same audio frames as with blocks of exactly one hop.
 */

#include "analysis/btrack_wrapper.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

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
static const int hop = 128;
static const double seconds = 12.0;

static std::vector<float> clickTrack(double bpm) {
    std::vector<float> audio(static_cast<size_t>(seconds * rate), 0.0f);
    uint32_t noise = 22222;
    int const clickLen = static_cast<int>(rate * 0.03);
    for (double click = rate * 0.5; click < audio.size(); click += rate * 60.0 / bpm) {
        for (int i = 0; i < clickLen && click + i < audio.size(); ++i) {
            noise = noise * 1664525u + 1013904223u;
            audio[static_cast<size_t>(click) + static_cast<size_t>(i)] =
                0.9f * static_cast<float>(std::exp(-6.0 * i / clickLen))
                * (static_cast<int32_t>(noise) / 2147483648.0f);
        }
    }
    return audio;
}

/** Feeds `audio` in blocks cycling through `blockSizes`; the beats' frames. */
static std::vector<int64_t> beatFrames(const std::vector<float>& audio,
                                       const std::vector<int>& blockSizes) {
    BTrackWrapper tracker(hop, hop * 2);
    std::vector<int64_t> beats;
    size_t pos = 0;
    for (size_t b = 0; pos < audio.size(); ++b) {
        int const want = blockSizes[b % blockSizes.size()];
        int const n = static_cast<int>(std::min<size_t>(static_cast<size_t>(want), audio.size() - pos));
        // A fresh buffer per block, exactly as large as JACK's would be.
        std::vector<float> block(audio.begin() + static_cast<long>(pos),
                                 audio.begin() + static_cast<long>(pos) + n);
        pos += static_cast<size_t>(n);
        if (auto samplesAfterBeat = tracker.processSamples(block.data(), n))
            beats.push_back(static_cast<int64_t>(pos) - *samplesAfterBeat);
    }
    return beats;
}

static void sameBeatsAs(const std::vector<int64_t>& reference, const std::vector<int>& blocks,
                        const char* name) {
    std::vector<int64_t> const beats = beatFrames(clickTrack(120.0), blocks);
    CHECK(beats == reference, "%s: %zu beats, one-hop blocks gave %zu (first %lld vs %lld)",
          name, beats.size(), reference.size(),
          beats.empty() ? -1LL : static_cast<long long>(beats.front()),
          reference.empty() ? -1LL : static_cast<long long>(reference.front()));
}

int main() {
    std::vector<int64_t> const reference = beatFrames(clickTrack(120.0), {hop});
    // 12 s at 120 BPM: BTrack needs a few seconds, then one beat every 0.5 s.
    CHECK(reference.size() >= 12, "reference found only %zu beats", reference.size());

    sameBeatsAs(reference, {64}, "blocks of 64");
    sameBeatsAs(reference, {256}, "blocks of 256");
    sameBeatsAs(reference, {512}, "blocks of 512");
    sameBeatsAs(reference, {1024}, "blocks of 1024");
    sameBeatsAs(reference, {2048}, "blocks of 2048");
    sameBeatsAs(reference, {4096}, "blocks of 4096");
    // The quantum changing while running, and sizes that are no multiple of the hop.
    sameBeatsAs(reference, {128, 128, 256, 1024, 64, 2048, 128}, "changing quantum");
    sameBeatsAs(reference, {100, 333, 37, 1000}, "odd sizes");

    if (failures == 0) std::printf("any_buffer_size_test: all passed\n");
    return failures == 0 ? 0 : 1;
}
