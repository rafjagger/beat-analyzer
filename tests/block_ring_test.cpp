/**
 * The ring between JACK's process thread and the beat thread takes a block of
 * any size: one longer than a slot is split over several, each with the audio
 * frame it ends at, so nothing is written past a slot and no frame is lost.
 */

#include "audio/block_ring.h"

#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

using BeatAnalyzer::Audio::BlockRing;

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

/** Pushes blocks of `size` and reads them back: samples and end frames intact. */
static void roundTrip(int size) {
    auto ring = std::make_unique<BlockRing>();
    std::vector<float> out;
    std::vector<int64_t> ends;
    int64_t frame = 0;
    for (int block = 0; block < 3; ++block) {
        std::vector<float> in(static_cast<size_t>(size));
        for (int i = 0; i < size; ++i) in[static_cast<size_t>(i)] = static_cast<float>(frame + i);
        frame += size;
        CHECK(ring->push(in.data(), size, frame), "block of %d did not fit", size);
        while (const BlockRing::Slot* slot = ring->front()) {
            CHECK(slot->frameCount > 0 && slot->frameCount <= BlockRing::SLOT_FRAMES,
                  "slot holds %d frames", slot->frameCount);
            out.insert(out.end(), slot->data, slot->data + slot->frameCount);
            ends.push_back(slot->endFrame);
            CHECK(slot->endFrame == static_cast<int64_t>(out.size()),
                  "block %d: slot ends at %lld, its last sample is frame %zu", size,
                  static_cast<long long>(slot->endFrame), out.size());
            ring->pop();
        }
    }
    CHECK(static_cast<int64_t>(out.size()) == frame, "block %d: %zu of %lld frames came through",
          size, out.size(), static_cast<long long>(frame));
    for (size_t i = 0; i < out.size(); ++i) {
        if (out[i] != static_cast<float>(i)) {
            CHECK(false, "block %d: frame %zu reads %f", size, i, static_cast<double>(out[i]));
            break;
        }
    }
}

/** A full ring drops the whole block rather than half of it. */
static void fullRingDropsWholeBlocks() {
    auto ring = std::make_unique<BlockRing>();
    std::vector<float> in(BlockRing::SLOT_FRAMES * 2, 1.0f);
    int64_t frame = 0;
    int pushed = 0;
    while (ring->push(in.data(), static_cast<int>(in.size()), frame += static_cast<int64_t>(in.size())))
        ++pushed;
    CHECK(pushed == (BlockRing::SLOTS - 1) / 2, "%d blocks of two slots fit", pushed);
    int slots = 0;
    while (ring->front()) { ring->pop(); ++slots; }
    CHECK(slots == pushed * 2, "%d slots for %d blocks", slots, pushed);
}

int main() {
    for (int size : {64, 128, 256, 512, 1024, 2048, 4096, 1000, 1500})
        roundTrip(size);
    fullRingDropsWholeBlocks();
    if (failures == 0) std::printf("block_ring_test: all passed\n");
    return failures == 0 ? 0 : 1;
}
