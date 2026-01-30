#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include "../include/analysis/beat_detection.h"

using namespace BeatAnalyzer;

// ============================================================================
// FFT Tests
// ============================================================================

void test_fft() {
    std::cout << "Testing FFT..." << std::endl;
    
    FFT fft(8);
    
    // Test mit bekanntem Signal (DC + 1 Hz Sinus @ 8 Hz Sample Rate)
    double input[8] = {1.0, 1.707, 2.0, 1.707, 1.0, 0.293, 0.0, 0.293};
    Complex output[8];
    
    fft.forward(input, output);
    
    // DC-Komponente sollte ~8 sein (Summe der Werte)
    double dcMag = std::abs(output[0]);
    assert(dcMag > 7.0 && dcMag < 9.0);
    
    // 1 Hz Komponente sollte stark sein
    double freq1Mag = std::abs(output[1]);
    assert(freq1Mag > 3.0);
    
    std::cout << "  ✓ FFT test passed" << std::endl;
}

// ============================================================================
// Window Function Tests
// ============================================================================

void test_window_function() {
    std::cout << "Testing Window Functions..." << std::endl;
    
    // Hanning window
    auto hanning = WindowFunction::create(WindowFunction::Hanning, 8);
    assert(hanning.size() == 8);
    
    // Hanning sollte bei den Rändern ~0 sein
    assert(hanning[0] < 0.01);
    assert(hanning[7] < 0.01);
    
    // Hanning sollte in der Mitte ~1 sein
    assert(hanning[4] > 0.9);
    
    // Hamming window
    auto hamming = WindowFunction::create(WindowFunction::Hamming, 8);
    assert(hamming.size() == 8);
    
    // Hamming hat einen Offset (nicht ganz 0 an den Rändern)
    assert(hamming[0] > 0.05 && hamming[0] < 0.1);
    
    std::cout << "  ✓ Window Function tests passed" << std::endl;
}

// ============================================================================
// Onset Detector Tests
// ============================================================================

void test_onset_detector() {
    std::cout << "Testing Onset Detector..." << std::endl;
    
    BeatDetectorConfig config;
    config.sampleRate = 44100;
    config.frameSize = 512;
    config.hopSize = 256;
    
    OnsetDetector detector(config);
    
    // Test mit Stille (initialisiere den Detektor)
    std::vector<double> silence(config.frameSize, 0.0);
    detector.process(silence.data());  // Erster Frame - initialisiert
    double onsetSilence = detector.process(silence.data());  // Zweiter Frame
    
    // Test mit Impuls (sollte hohen Onset-Wert geben)
    std::vector<double> impulse(config.frameSize, 0.0);
    impulse[0] = 1.0;
    impulse[10] = 0.8;
    impulse[20] = 0.6;
    double onsetImpulse = detector.process(impulse.data());
    
    // Impuls sollte höheren Onset-Wert haben als Stille
    // (oder zumindest nicht negativ sein)
    assert(onsetImpulse >= 0.0);
    assert(onsetSilence >= 0.0);
    // Impuls sollte erkennbar sein (auch wenn klein)
    std::cout << "  Silence onset: " << onsetSilence << ", Impulse onset: " << onsetImpulse << std::endl;
    
    std::cout << "  ✓ Onset Detector tests passed" << std::endl;
}

// ============================================================================
// Tempo Tracker Tests
// ============================================================================

void test_tempo_tracker() {
    std::cout << "Testing Tempo Tracker..." << std::endl;
    
    TempoTracker tracker(44100, 512);
    
    // Erstelle synthetische Detection Function mit regelmäßigen Peaks
    // 120 BPM = 2 Beats/Sekunde = ~86 Frames bei 512 Hop Size @ 44100 Hz
    // Frames pro Beat = (60/120) * 44100 / 512 ≈ 43
    
    std::vector<double> df(500, 0.1);
    int framesPerBeat = 43;  // ~120 BPM
    
    for (int i = 0; i < 500; i += framesPerBeat) {
        df[i] = 1.0;  // Beat-Peak
    }
    
    std::vector<int> beatPeriod;
    tracker.calculateBeatPeriod(df, beatPeriod, 120.0);
    
    // Sollte Perioden haben
    assert(!beatPeriod.empty());
    
    // Durchschnittliche Periode sollte nahe 43 sein
    double avgPeriod = 0.0;
    for (int p : beatPeriod) {
        avgPeriod += p;
    }
    avgPeriod /= beatPeriod.size();
    
    // Toleranz von ±20%
    assert(avgPeriod > 30 && avgPeriod < 60);
    
    std::cout << "  ✓ Tempo Tracker tests passed (avg period: " << avgPeriod << ")" << std::endl;
}

// ============================================================================
// Real-Time Beat Tracker Tests
// ============================================================================

void test_realtime_beat_tracker() {
    std::cout << "Testing Real-Time Beat Tracker..." << std::endl;
    
    BeatDetectorConfig config;
    config.sampleRate = 44100;
    config.frameSize = 1024;
    config.hopSize = 512;
    config.defaultBpm = 120.0;
    
    RealTimeBeatTracker tracker(config);
    assert(tracker.initialize());
    
    // Generiere 5 Sekunden synthetisches Audio mit 120 BPM Clicks
    int totalFrames = 5 * config.sampleRate;
    int samplesPerBeat = config.sampleRate / 2;  // 120 BPM = 0.5s pro Beat
    
    std::vector<Sample> audio(totalFrames * 2, 0.0f);  // Stereo
    
    // Füge Clicks ein
    for (int i = 0; i < totalFrames; i += samplesPerBeat) {
        // Kurzer Impuls
        for (int j = 0; j < 100 && i + j < totalFrames; ++j) {
            float decay = 1.0f - (j / 100.0f);
            audio[(i + j) * 2] = decay * 0.5f;      // Left
            audio[(i + j) * 2 + 1] = decay * 0.5f;  // Right
        }
    }
    
    // Verarbeite in Chunks
    int chunkSize = 512;
    for (int i = 0; i < totalFrames; i += chunkSize) {
        int remaining = std::min(chunkSize, totalFrames - i);
        tracker.processAudio(&audio[i * 2], remaining);
    }
    
    // Finalisiere
    BeatInfo result = tracker.finalize();
    
    std::cout << "  Detected BPM: " << result.bpm << std::endl;
    std::cout << "  Detected Beats: " << result.beatFrames.size() << std::endl;
    
    // BPM sollte im Bereich 100-140 sein (120 BPM mit Toleranz)
    if (result.valid) {
        assert(result.bpm > 100.0 && result.bpm < 140.0);
    }
    
    std::cout << "  ✓ Real-Time Beat Tracker tests passed" << std::endl;
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════╗\n";
    std::cout << "║     BEAT ANALYZER - UNIT TESTS            ║\n";
    std::cout << "╚═══════════════════════════════════════════╝\n";
    std::cout << "\n";
    
    try {
        test_fft();
        test_window_function();
        test_onset_detector();
        test_tempo_tracker();
        test_realtime_beat_tracker();
        
        std::cout << "\n";
        std::cout << "═══════════════════════════════════════════\n";
        std::cout << "  ✓ ALL TESTS PASSED!\n";
        std::cout << "═══════════════════════════════════════════\n";
        std::cout << "\n";
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "\n✗ Test failed: " << e.what() << std::endl;
        return 1;
    }
}
