// Real-time priority for the whole process put all of beat-analyzer's threads
// -- OSC, VU sender, BTrack -- above JACK's process threads, and JACK reported
// late clients until it was gone (a3nuc1, 2026-10-06). Only the JACK process
// thread runs real-time, and JACK gives it its priority.
//
// Not even as a comment: a commented-out line is the one that gets switched
// back on.
#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

static void test_the_unit_raises_no_whole_process()
{
    std::ifstream unit(BEAT_ANALYZER_SOURCE_DIR "/beat-analyzer.service");
    assert(unit.good());

    std::string line;
    int lines = 0;
    while (std::getline(unit, line)) {
        ++lines;
        if (line.find("CPUSchedulingPolicy") != std::string::npos
            || line.find("CPUSchedulingPriority") != std::string::npos
            || line.find("LimitRTPRIO") != std::string::npos) {
            std::cerr << "  ✗ " << line << std::endl;
            assert(false);
        }
    }
    assert(lines > 0);
    std::cout << "  ✓ beat-analyzer.service sets no real-time scheduling" << std::endl;
}

int main()
{
    test_the_unit_raises_no_whole_process();
    return 0;
}
