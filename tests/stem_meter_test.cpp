// One meter per stem pair: the louder side, peak and RMS each on their own
// (issue a3-system#71).
#include <cassert>
#include <iostream>

#include "../include/analysis/stem_meter.h"

using namespace BeatAnalyzer::Analysis;

static void test_the_louder_side_wins()
{
    auto level = louder({0.5f, 0.2f}, {0.3f, 0.4f});
    assert(level.peak == 0.5f && level.rms == 0.4f);
    std::cout << "  ✓ peak and rms each from the louder side" << std::endl;
}

static void test_silence_is_silence()
{
    auto level = louder({0.0f, 0.0f}, {0.0f, 0.0f});
    assert(level.peak == 0.0f && level.rms == 0.0f);
    std::cout << "  ✓ silence" << std::endl;
}

int main()
{
    std::cout << "Stem meter tests" << std::endl;
    test_the_louder_side_wins();
    test_silence_is_silence();
    return 0;
}
