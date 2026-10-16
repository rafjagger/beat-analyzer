#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

namespace BeatAnalyzer {
namespace Audio {

/**
 * Lock-free single-producer/single-consumer ring: JACK's process thread
 * pushes its blocks, the beat thread takes them. Nothing allocates.
 */
class BlockRing {
public:
    static constexpr int SLOTS = 64;
    static constexpr int SLOT_FRAMES = 1024;

    struct Slot {
        float data[SLOT_FRAMES];
        int frameCount = 0;
        /** Audio-Frame am Ende dieses Puffers: wo ein Beat darin liegt. */
        int64_t endFrame = 0;
    };

    /**
     * Producer. `endFrame` is the audio frame after the block's last sample.
     * A block longer than a slot is split over several; when the ring has no
     * room for all of it, nothing is pushed and the block is dropped whole.
     */
    bool push(const float* samples, int frameCount, int64_t endFrame) {
        int const slotsNeeded = (frameCount + SLOT_FRAMES - 1) / SLOT_FRAMES;
        if (slotsNeeded > freeSlots()) return false;
        int w = m_wpos.load(std::memory_order_relaxed);
        int64_t const startFrame = endFrame - frameCount;
        for (int done = 0; done < frameCount;) {
            int const n = frameCount - done < SLOT_FRAMES ? frameCount - done : SLOT_FRAMES;
            Slot& slot = m_slots[w];
            std::memcpy(slot.data, samples + done, static_cast<size_t>(n) * sizeof(float));
            done += n;
            slot.frameCount = n;
            slot.endFrame = startFrame + done;
            w = (w + 1) & MASK;
        }
        m_wpos.store(w, std::memory_order_release);
        return true;
    }

    /** Consumer: the oldest slot, or nullptr when the ring is empty. */
    const Slot* front() const {
        int const r = m_rpos.load(std::memory_order_relaxed);
        if (r == m_wpos.load(std::memory_order_acquire)) return nullptr;
        return &m_slots[r];
    }

    /** Consumer: releases the slot front() returned. */
    void pop() {
        int const r = m_rpos.load(std::memory_order_relaxed);
        m_rpos.store((r + 1) & MASK, std::memory_order_release);
    }

private:
    int freeSlots() const {
        int const w = m_wpos.load(std::memory_order_relaxed);
        int const r = m_rpos.load(std::memory_order_acquire);
        return SLOTS - 1 - ((w - r) & MASK);
    }

    static constexpr int MASK = SLOTS - 1;
    static_assert((SLOTS & MASK) == 0, "SLOTS must be a power of two");

    Slot m_slots[SLOTS];
    alignas(64) std::atomic<int> m_wpos{0};
    alignas(64) std::atomic<int> m_rpos{0};
};

} // namespace Audio
} // namespace BeatAnalyzer
