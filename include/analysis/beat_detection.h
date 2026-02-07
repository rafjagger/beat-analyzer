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
    int frameSize = 512;            // FFT Window Size (kleiner = tighter timing)
    int hopSize = 256;              // Step Size (kleiner = höhere Zeitauflösung)
    double minBpm = 60.0;
    double maxBpm = 200.0;
    double defaultBpm = 120.0;
    bool adaptiveWhitening = false;
    double dbRise = 3.0;
    
    // Auto Gain Control
    bool agcEnabled = true;         // Dynamische Eingangsverstärkung
    float agcTargetPeak = 0.7f;     // Ziel-Pegelspitze (linear, 0-1)
    
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
    std::vector<double> m_freqWeights;  // Frequenzgewichtung (Bass betont)
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

// ============================================================================
// Beat Event Detector - erkennt echte Beats aus Onset-Funktion
// ============================================================================

class BeatEventDetector {
public:
    BeatEventDetector(int sampleRate, int hopSize, double maxBpm);
    
    // Verarbeite neuen Onset-Wert, gibt true zurück wenn Beat erkannt
    bool process(double onsetValue);
    
    // Reset
    void reset();
    
private:
    // Adaptive Threshold
    double m_threshold = 0.0;
    double m_adaptiveThreshold = 0.0;
    double m_runningMean = 0.0;
    double m_runningDev = 0.0;
    static constexpr double THRESHOLD_DECAY = 0.9;
    static constexpr double THRESHOLD_RISE = 1.5;
    
    // Peak Detection
    double m_prevOnset = 0.0;
    double m_prevPrevOnset = 0.0;
    
    // Minimum Zeit zwischen Beats (verhindert Doppel-Trigger)
    int m_sampleRate;
    int m_minBeatInterval;  // Frames
    int m_framesSinceLastBeat = 0;
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
    
    // Verarbeite Audio-Block (bereits Mono)
    void processMonoAudio(const Sample* monoInput, int frameCount);
    
    // Finalisiere Analyse
    BeatInfo finalize();
    
    // Aktuelle BPM Schätzung (für Echtzeit-Display)
    double getCurrentBpm() const { return m_currentBpm; }
    
    // Tap-Referenz BPM setzen: steuert die Rayleigh-Gewichtung in der ACF
    // Damit wählt die ACF die richtige Harmonische (z.B. 90 statt 130)
    // 0.0 = kein Hint, nutze (minBpm+maxBpm)/2
    void setReferenceBpm(double bpm) { m_referenceBpmHint = bpm; }
    double getReferenceBpm() const { return m_referenceBpmHint; }
    
    // Wurde im letzten processAudio() ein Beat erkannt?
    bool hasBeatOccurred() const { return m_beatOccurred; }
    
    // Onset-Stärke des letzten erkannten Beats (für Downbeat-Analyse)
    double getLastBeatOnsetStrength() const { return m_lastBeatOnsetStrength; }
    
    // Reset
    void reset();
    
private:
    BeatDetectorConfig m_config;
    std::unique_ptr<OnsetDetector> m_onsetDetector;
    std::unique_ptr<TempoTracker> m_tempoTracker;
    std::unique_ptr<BeatEventDetector> m_beatEventDetector;
    bool m_beatOccurred = false;
    double m_lastBeatOnsetStrength = 0.0;  // Onset-Stärke des letzten erkannten Beats
    
    // Buffering
    std::vector<double> m_monoBuffer;
    std::vector<double> m_frameBuffer;
    std::vector<double> m_detectionFunction;
    size_t m_bufferWritePos;
    
    double m_currentBpm;
    double m_referenceBpmHint = 0.0;  // Tap-Referenz für ACF Rayleigh-Gewichtung
    int64_t m_totalFramesProcessed;
    
    // Auto Gain Control
    float m_agcGain = 1.0f;         // Aktueller Verstärkungsfaktor
    float m_agcPeakTracker = 0.0f;  // Laufender Peak-Tracker
    
    // Tempo-vorhergesagter Onset-Frame-Zähler
    int64_t m_lastBeatOnsetFrame = 0;  // Letzter erkannter Beat in DF-Frames
    
    // Downmix Stereo zu Mono
    void downmixToMono(const Sample* stereo, double* mono, int frames);
    
    // Interne Mono-Verarbeitung (gemeinsam für stereo und mono Eingang)
    void processMonoInternal(const double* monoInput, int frameCount);
    
    // Echtzeit BPM Update
    void updateRealtimeBpm();
};

} // namespace BeatAnalyzer
