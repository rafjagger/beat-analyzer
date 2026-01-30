#include <iostream>
#include <cassert>
#include <cmath>
#include "../include/audio/audio_buffer.h"
#include "../include/analysis/beat_tracker.h"

using namespace BeatAnalyzer::Audio;
using namespace BeatAnalyzer::Analysis;

void test_audio_buffer() {
    std::cout << "Testing AudioBuffer..." << std::endl;
    
    AudioBuffer buffer(2, 1024);  // 2 channels, 1024 frame capacity
    
    // Test write/read
    float testData[4] = {1.0f, 2.0f, 3.0f, 4.0f};  // 2 frames
    buffer.write(testData, 2);
    
    assert(buffer.available() == 2);
    
    float readData[4];
    buffer.read(readData, 2);
    
    assert(readData[0] == 1.0f && readData[1] == 2.0f);
    assert(buffer.available() == 0);
    
    std::cout << "  ✓ AudioBuffer test passed" << std::endl;
}

void test_beat_tracker() {
    std::cout << "Testing BeatTracker..." << std::endl;
    
    BeatTracker tracker(SampleRate(44100));
    
    // Create beat info
    BeatInfo beats;
    beats.bpm = Bpm(120.0);
    beats.beat_frames = {
        FramePos(0),
        FramePos(22050),      // 0.5 sec @ 44.1kHz
        FramePos(44100),      // 1.0 sec
        FramePos(66150)       // 1.5 sec
    };
    beats.confidence = 0.95;
    beats.valid = true;
    
    tracker.updateBeats(beats);
    
    // Test beat number
    int beatNum = tracker.getBeatNumber(FramePos(22050));
    assert(beatNum >= 0 && beatNum < 4);
    
    // Test phase
    double phase = tracker.getBeatPhase(FramePos(11025));  // Half-way
    assert(phase >= 0.0 && phase <= 1.0);
    
    // Test next beat
    FramePos nextBeat = tracker.getNextBeat(FramePos(0));
    assert(nextBeat.frame > 0);
    
    std::cout << "  ✓ BeatTracker test passed" << std::endl;
}

void test_audio_types() {
    std::cout << "Testing AudioTypes..." << std::endl;
    
    // Test FramePos
    FramePos fp(44100);
    double seconds = fp.toSeconds(44100);
    assert(std::abs(seconds - 1.0) < 0.001);
    
    // Test SampleRate
    SampleRate sr(48000);
    assert(sr.value == 48000);
    assert(sr == 48000);
    
    // Test Bpm
    Bpm bpm(120.0);
    assert(bpm.isValid());
    assert(bpm == 120.0);
    
    // Test ChannelCount
    ChannelCount cc = ChannelCount::stereo();
    assert(cc.value == 2);
    
    std::cout << "  ✓ AudioTypes test passed" << std::endl;
}

int main() {
    std::cout << "\n=== Beat Analyzer Unit Tests ===" << std::endl;
    std::cout << std::endl;
    
    try {
        test_audio_types();
        test_audio_buffer();
        test_beat_tracker();
        
        std::cout << std::endl;
        std::cout << "=== All tests passed! ===" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
}
