// The analyzer sends raw peaks: per tick the highest sample since the last
// send, no fall and no hold. Every display applies the shared ballistics
// (attack immediate, release 20 dB/s, hold 1.5 s) on its own clock, so a fall
// here would be applied twice (.claude/notes/meter-ballistics.md, 2026-10-07).
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "../include/analysis/vu_meter.h"

using namespace BeatAnalyzer;

static std::vector<float> block(float level, int frames = 256)
{
    std::vector<float> samples(frames, 0.0f);
    samples[frames / 2] = level;
    samples[frames / 2 + 1] = -level * 0.5f;
    return samples;
}

static void feed(VuMeter& meter, const std::vector<float>& samples)
{
    meter.processMono(samples.data(), static_cast<int>(samples.size()));
}

static bool near(float a, float b) { return std::fabs(a - b) < 1e-6f; }

static void test_a_loud_block_shows_once_at_full_level()
{
    VuMeter meter(48000);
    feed(meter, block(1.0f));
    assert(near(meter.takePeakLinear(), 1.0f));
    std::cout << "  ✓ a loud block shows at full level" << std::endl;
}

static void test_the_next_tick_shows_the_new_block_not_a_fall()
{
    VuMeter meter(48000);
    feed(meter, block(1.0f));
    meter.takePeakLinear();
    feed(meter, block(0.1f));
    assert(near(meter.takePeakLinear(), 0.1f));
    std::cout << "  ✓ the next tick shows the new block's level" << std::endl;
}

static void test_silence_after_a_peak_reads_zero()
{
    VuMeter meter(48000);
    feed(meter, block(1.0f));
    meter.takePeakLinear();
    feed(meter, block(0.0f));
    assert(near(meter.takePeakLinear(), 0.0f));
    std::cout << "  ✓ silence after a peak reads zero" << std::endl;
}

static void test_the_highest_block_between_two_sends_wins()
{
    VuMeter meter(48000);
    feed(meter, block(0.2f));
    feed(meter, block(0.9f));
    feed(meter, block(0.3f));
    assert(near(meter.takePeakLinear(), 0.9f));
    std::cout << "  ✓ the highest block between two sends wins" << std::endl;
}

static void test_a_send_without_audio_reads_zero()
{
    VuMeter meter(48000);
    feed(meter, block(0.7f));
    meter.takePeakLinear();
    assert(near(meter.takePeakLinear(), 0.0f));
    std::cout << "  ✓ a send without new audio reads zero" << std::endl;
}

int main()
{
    std::cout << "VuMeter raw peak tests" << std::endl;
    test_a_loud_block_shows_once_at_full_level();
    test_the_next_tick_shows_the_new_block_not_a_fall();
    test_silence_after_a_peak_reads_zero();
    test_the_highest_block_between_two_sends_wins();
    test_a_send_without_audio_reads_zero();
    std::cout << "All VuMeter tests passed" << std::endl;
    return 0;
}
