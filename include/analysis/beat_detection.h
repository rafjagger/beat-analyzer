#pragma once

/**
 * Beat Analyzer - Eigenständige Anwendung
 * 
 * Dieser Code ist von Grund auf neu geschrieben und nutzt
 * nur die KONZEPTE aus Mixxx als Vorlage:
 * 
 * - Onset Detection Function (Spectral Difference)
 * - Autocorrelation-based Tempo Tracking
 * - Comb Filter Bank für Periodizität
 * - Viterbi Decoding für optimalen Beat-Pfad
 * 
 * KEINE Mixxx-Abhängigkeiten!
 */

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <complex>
#include <memory>

namespace BeatAnalyzer {

// Grundtypen
using Sample = float;
using SampleBuffer = std::vector<Sample>;

// FFT Ergebnistyp
using Complex = std::complex<double>;
using ComplexBuffer = std::vector<Complex>;

/**
 * Konfiguration für Beat Detection
 */
struct BeatDetectorConfig {
    int sampleRate = 44100;
    int frameSize = 1024;           // FFT Window Size
    int hopSize = 512;              // Step Size
    double minBpm = 60.0;
    double maxBpm = 200.0;
    double defaultBpm = 120.0;
    bool adaptiveWhitening = false;
    double dbRise = 3.0;
    
    // Berechne Step-Size in Sekunden
    double hopSizeSeconds() const {
        return static_cast<double>(hopSize) / sampleRate;
    }
};

/**
 * Beat Information
 */
struct BeatInfo {
    std::vector<int64_t> beatFrames;     // Beat Positionen in Samples
    double bpm;                           // Erkannte BPM
    double confidence;                    // Konfidenz (0.0 - 1.0)
    bool valid;
    
    BeatInfo() : bpm(0.0), confidence(0.0), valid(false) {}
};

/**
 * Einfache FFT Implementation (Cooley-Tukey)
 */
class FFT {
public:
    explicit FFT(int size);
    
    void forward(const double* input, Complex* output);
    void inverse(const Complex* input, double* output);
    
private:
    int m_size;
    int m_log2Size;
    std::vector<Complex> m_twiddleFactors;
    std::vector<int> m_bitReversed;
    
    void computeTwiddleFactors();
    void computeBitReversal();
    int reverseBits(int x, int bits);
};

/**
 * Fenster-Funktionen
 */
class WindowFunction {
public:
    enum Type { Hanning, Hamming, Blackman, Rectangle };
    
    static std::vector<double> create(Type type, int size);
    static void apply(const std::vector<double>& window, 
                     const double* input, double* output);
};

/**
 * Onset Detection Function
 * Erkennt Onsets/Transienten im Audio basierend auf
 * Spectral Difference (Complex Domain)
 */
class OnsetDetector {
public:
    explicit OnsetDetector(const BeatDetectorConfig& config);
    ~OnsetDetector();
    
    // Verarbeite ein Audio-Frame
    double process(const double* frame);
    
    // Reset den internen Zustand
    void reset();
    
private:
    BeatDetectorConfig m_config;
    std::unique_ptr<FFT> m_fft;
    std::vector<double> m_window;
    std::vector<double> m_windowed;
    std::vector<double> m_magnitude;
    std::vector<double> m_phase;
    std::vector<double> m_prevMagnitude;
    std::vector<double> m_prevPhase;
    std::vector<double> m_prevPrevPhase;
    ComplexBuffer m_spectrum;
    
    double computeSpectralDifference();
    double computeComplexSpectralDifference();
};

/**
 * Tempo Tracker
 * Verwendet Autocorrelation + Comb Filter Bank + Viterbi Decoding
 */
class TempoTracker {
public:
    explicit TempoTracker(int sampleRate, int hopSize);
    
    // Berechne optimale Beat-Perioden aus Detection Function
    void calculateBeatPeriod(const std::vector<double>& df,
                            std::vector<int>& beatPeriod,
                            double inputTempo = 120.0);
    
    // Berechne Beat-Positionen
    void calculateBeats(const std::vector<double>& df,
                       const std::vector<int>& beatPeriod,
                       std::vector<double>& beats);
    
private:
    int m_sampleRate;
    int m_hopSize;
    
    // Filter die Detection Function
    void filterDF(std::vector<double>& df);
    
    // Autocorrelation
    void autocorrelate(const std::vector<double>& input,
                      std::vector<double>& output);
    
    // Resonator Comb Filter
    void combFilter(const std::vector<double>& acf,
                   const std::vector<double>& weights,
                   std::vector<double>& rcf);
    
    // Viterbi Decoding
    void viterbiDecode(const std::vector<std::vector<double>>& rcfMatrix,
                      const std::vector<double>& weights,
                      std::vector<int>& beatPeriod);
    
    // Adaptive Threshold
    void adaptiveThreshold(std::vector<double>& data);
};

/**
 * Real-Time Beat Tracker
 * Kombiniert Onset Detection + Tempo Tracking für Echtzeit-Analyse
 */
class RealTimeBeatTracker {
public:
    explicit RealTimeBeatTracker(const BeatDetectorConfig& config);
    
    // Initialisiere den Tracker
    bool initialize();
    
    // Verarbeite Audio-Block (Stereo zu Mono)
    void processAudio(const Sample* stereoInput, int frameCount);
    
    // Finalisiere Analyse
    BeatInfo finalize();
    
    // Aktuelle BPM Schätzung (für Echtzeit-Display)
    double getCurrentBpm() const { return m_currentBpm; }
    
    // Reset
    void reset();
    
private:
    BeatDetectorConfig m_config;
    std::unique_ptr<OnsetDetector> m_onsetDetector;
    std::unique_ptr<TempoTracker> m_tempoTracker;
    
    // Buffering
    std::vector<double> m_monoBuffer;
    std::vector<double> m_frameBuffer;
    std::vector<double> m_detectionFunction;
    size_t m_bufferWritePos;
    
    double m_currentBpm;
    int64_t m_totalFramesProcessed;
    
    // Downmix Stereo zu Mono
    void downmixToMono(const Sample* stereo, double* mono, int frames);
    
    // Echtzeit BPM Update
    void updateRealtimeBpm();
};

} // namespace BeatAnalyzer
