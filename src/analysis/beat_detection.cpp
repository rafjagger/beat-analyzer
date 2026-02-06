/**
 * Beat Analyzer - Eigenständige Beat Detection Implementierung
 * 
 * Basiert auf den KONZEPTEN von:
 * - Queen Mary University DSP Library (Algorithmus-Ideen)
 * - Onset Detection via Spectral Difference
 * - Tempo Tracking via Autocorrelation + Viterbi
 * 
 * Dieser Code ist komplett eigenständig und hat
 * KEINE externen Abhängigkeiten außer Standard C++!
 */

#include "analysis/beat_detection.h"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>

namespace BeatAnalyzer {

// ============================================================================
// FFT Implementation (Cooley-Tukey Radix-2)
// ============================================================================

FFT::FFT(int size) : m_size(size) {
    // Prüfe ob Potenz von 2
    m_log2Size = 0;
    int temp = size;
    while (temp > 1) {
        temp >>= 1;
        m_log2Size++;
    }
    
    computeTwiddleFactors();
    computeBitReversal();
}

void FFT::computeTwiddleFactors() {
    m_twiddleFactors.resize(m_size / 2);
    const double pi = 3.14159265358979323846;
    
    for (int i = 0; i < m_size / 2; ++i) {
        double angle = -2.0 * pi * i / m_size;
        m_twiddleFactors[i] = Complex(std::cos(angle), std::sin(angle));
    }
}

void FFT::computeBitReversal() {
    m_bitReversed.resize(m_size);
    for (int i = 0; i < m_size; ++i) {
        m_bitReversed[i] = reverseBits(i, m_log2Size);
    }
}

int FFT::reverseBits(int x, int bits) {
    int result = 0;
    for (int i = 0; i < bits; ++i) {
        result = (result << 1) | (x & 1);
        x >>= 1;
    }
    return result;
}

void FFT::forward(const double* input, Complex* output) {
    // Bit-reversal permutation
    for (int i = 0; i < m_size; ++i) {
        output[m_bitReversed[i]] = Complex(input[i], 0.0);
    }
    
    // Cooley-Tukey iterative FFT
    for (int stage = 1; stage <= m_log2Size; ++stage) {
        int m = 1 << stage;
        int halfM = m / 2;
        int step = m_size / m;
        
        for (int k = 0; k < m_size; k += m) {
            for (int j = 0; j < halfM; ++j) {
                Complex t = m_twiddleFactors[j * step] * output[k + j + halfM];
                Complex u = output[k + j];
                output[k + j] = u + t;
                output[k + j + halfM] = u - t;
            }
        }
    }
}

void FFT::inverse(const Complex* input, double* output) {
    // Konjugiere, FFT, konjugiere, normalisiere
    std::vector<Complex> temp(m_size);
    for (int i = 0; i < m_size; ++i) {
        temp[i] = std::conj(input[i]);
    }
    
    std::vector<Complex> result(m_size);
    std::vector<double> realPart(m_size);
    for (int i = 0; i < m_size; ++i) {
        realPart[i] = temp[i].real();
    }
    
    forward(realPart.data(), result.data());
    
    for (int i = 0; i < m_size; ++i) {
        output[i] = result[i].real() / m_size;
    }
}

// ============================================================================
// Window Function
// ============================================================================

std::vector<double> WindowFunction::create(Type type, int size) {
    std::vector<double> window(size);
    const double pi = 3.14159265358979323846;
    
    switch (type) {
        case Hanning:
            for (int i = 0; i < size; ++i) {
                window[i] = 0.5 * (1.0 - std::cos(2.0 * pi * i / (size - 1)));
            }
            break;
            
        case Hamming:
            for (int i = 0; i < size; ++i) {
                window[i] = 0.54 - 0.46 * std::cos(2.0 * pi * i / (size - 1));
            }
            break;
            
        case Blackman:
            for (int i = 0; i < size; ++i) {
                window[i] = 0.42 - 0.5 * std::cos(2.0 * pi * i / (size - 1))
                          + 0.08 * std::cos(4.0 * pi * i / (size - 1));
            }
            break;
            
        case Rectangle:
        default:
            std::fill(window.begin(), window.end(), 1.0);
            break;
    }
    
    return window;
}

void WindowFunction::apply(const std::vector<double>& window,
                          const double* input, double* output) {
    for (size_t i = 0; i < window.size(); ++i) {
        output[i] = input[i] * window[i];
    }
}

// ============================================================================
// Onset Detector
// ============================================================================

OnsetDetector::OnsetDetector(const BeatDetectorConfig& config)
    : m_config(config) {
    
    int halfSize = config.frameSize / 2 + 1;
    
    m_fft = std::make_unique<FFT>(config.frameSize);
    m_window = WindowFunction::create(WindowFunction::Hanning, config.frameSize);
    m_windowed.resize(config.frameSize);
    m_magnitude.resize(halfSize);
    m_phase.resize(halfSize);
    m_prevMagnitude.resize(halfSize, 0.0);
    m_prevPhase.resize(halfSize, 0.0);
    m_prevPrevPhase.resize(halfSize, 0.0);
    m_spectrum.resize(config.frameSize);
    
    // Frequenzgewichtung: Bass (Kick/Snare) betonen, Höhen dämpfen
    // Bin-Frequenz = bin * sampleRate / frameSize
    m_freqWeights.resize(halfSize);
    double binFreqStep = static_cast<double>(config.sampleRate) / config.frameSize;
    for (int i = 0; i < halfSize; ++i) {
        double freq = i * binFreqStep;
        if (freq < 30.0) {
            // Sub-Bass: moderate Gewichtung (Kick-Fundamental)
            m_freqWeights[i] = 1.5;
        } else if (freq < 200.0) {
            // Bass: starke Gewichtung (Kick/Bass-Transienten)
            m_freqWeights[i] = 3.0;
        } else if (freq < 500.0) {
            // Low-Mid: gute Gewichtung (Snare Body)
            m_freqWeights[i] = 2.0;
        } else if (freq < 4000.0) {
            // Mid: normal (Snare Click, Hi-Hat)
            m_freqWeights[i] = 1.0;
        } else if (freq < 8000.0) {
            // High-Mid: reduziert
            m_freqWeights[i] = 0.5;
        } else {
            // Höhen: stark reduziert (Rauschen, Becken-Rauschen)
            m_freqWeights[i] = 0.2;
        }
    }
}

OnsetDetector::~OnsetDetector() = default;

double OnsetDetector::process(const double* frame) {
    // Fensterung
    WindowFunction::apply(m_window, frame, m_windowed.data());
    
    // FFT
    m_fft->forward(m_windowed.data(), m_spectrum.data());
    
    // Magnitude und Phase extrahieren
    int halfSize = m_config.frameSize / 2 + 1;
    for (int i = 0; i < halfSize; ++i) {
        m_magnitude[i] = std::abs(m_spectrum[i]);
        m_phase[i] = std::arg(m_spectrum[i]);
    }
    
    // Onset Detection berechnen
    double onset = computeComplexSpectralDifference();
    
    // History updaten
    std::copy(m_prevPhase.begin(), m_prevPhase.end(), m_prevPrevPhase.begin());
    std::copy(m_phase.begin(), m_phase.end(), m_prevPhase.begin());
    std::copy(m_magnitude.begin(), m_magnitude.end(), m_prevMagnitude.begin());
    
    return onset;
}

double OnsetDetector::computeSpectralDifference() {
    double sum = 0.0;
    int halfSize = m_config.frameSize / 2 + 1;
    
    for (int i = 0; i < halfSize; ++i) {
        double diff = m_magnitude[i] - m_prevMagnitude[i];
        // Half-wave rectification (nur positive Unterschiede)
        if (diff > 0) {
            sum += diff * diff;
        }
    }
    
    return std::sqrt(sum);
}

double OnsetDetector::computeComplexSpectralDifference() {
    // Complex Spectral Difference mit Frequenzgewichtung
    // Bass-Frequenzen werden betont, da dort die rhythmischen Transienten liegen
    double sum = 0.0;
    int halfSize = m_config.frameSize / 2 + 1;
    
    for (int i = 0; i < halfSize; ++i) {
        // Erwartete Phase basierend auf vorherigen Frames (lineare Extrapolation)
        double expectedPhase = 2.0 * m_prevPhase[i] - m_prevPrevPhase[i];
        
        // Vorhergesagtes Spektrum
        double predictedReal = m_prevMagnitude[i] * std::cos(expectedPhase);
        double predictedImag = m_prevMagnitude[i] * std::sin(expectedPhase);
        
        // Aktuelles Spektrum
        double currReal = m_magnitude[i] * std::cos(m_phase[i]);
        double currImag = m_magnitude[i] * std::sin(m_phase[i]);
        
        // Gewichtete Euclidean distance
        double diffReal = currReal - predictedReal;
        double diffImag = currImag - predictedImag;
        double dist = diffReal * diffReal + diffImag * diffImag;
        
        // Frequenzgewichtung anwenden
        sum += dist * m_freqWeights[i];
    }
    
    return std::sqrt(sum);
}

void OnsetDetector::reset() {
    std::fill(m_prevMagnitude.begin(), m_prevMagnitude.end(), 0.0);
    std::fill(m_prevPhase.begin(), m_prevPhase.end(), 0.0);
    std::fill(m_prevPrevPhase.begin(), m_prevPrevPhase.end(), 0.0);
}

// ============================================================================
// Tempo Tracker
// ============================================================================

TempoTracker::TempoTracker(int sampleRate, int hopSize)
    : m_sampleRate(sampleRate), m_hopSize(hopSize) {
}

void TempoTracker::filterDF(std::vector<double>& df) {
    // Butterworth Tiefpassfilter (2. Ordnung, 0.4 normalisierte Frequenz)
    // Koeffizienten aus MATLAB: [b,a] = butter(2, 0.4)
    const double a[] = {1.0, -0.3695, 0.1958};
    const double b[] = {0.2066, 0.4131, 0.2066};
    
    std::vector<double> filtered(df.size());
    
    double inp1 = 0.0, inp2 = 0.0;
    double out1 = 0.0, out2 = 0.0;
    
    // Forward pass
    for (size_t i = 0; i < df.size(); ++i) {
        filtered[i] = b[0] * df[i] + b[1] * inp1 + b[2] * inp2
                    - a[1] * out1 - a[2] * out2;
        inp2 = inp1;
        inp1 = df[i];
        out2 = out1;
        out1 = filtered[i];
    }
    
    // Zeit-Umkehr
    std::reverse(filtered.begin(), filtered.end());
    std::copy(filtered.begin(), filtered.end(), df.begin());
    
    inp1 = inp2 = out1 = out2 = 0.0;
    
    // Backward pass
    for (size_t i = 0; i < df.size(); ++i) {
        filtered[i] = b[0] * df[i] + b[1] * inp1 + b[2] * inp2
                    - a[1] * out1 - a[2] * out2;
        inp2 = inp1;
        inp1 = df[i];
        out2 = out1;
        out1 = filtered[i];
    }
    
    // Wieder zurück-umkehren
    std::reverse(filtered.begin(), filtered.end());
    std::copy(filtered.begin(), filtered.end(), df.begin());
}

void TempoTracker::adaptiveThreshold(std::vector<double>& data) {
    if (data.empty()) return;
    
    // Median-basierte Threshold
    std::vector<double> sorted = data;
    std::sort(sorted.begin(), sorted.end());
    double median = sorted[sorted.size() / 2];
    
    for (double& val : data) {
        val = std::max(0.0, val - median);
    }
}

void TempoTracker::autocorrelate(const std::vector<double>& input,
                                std::vector<double>& output) {
    int len = static_cast<int>(input.size());
    output.resize(len);
    
    for (int lag = 0; lag < len; ++lag) {
        double sum = 0.0;
        for (int n = 0; n < len - lag; ++n) {
            sum += input[n] * input[n + lag];
        }
        output[lag] = sum / (len - lag);
    }
}

void TempoTracker::combFilter(const std::vector<double>& acf,
                             const std::vector<double>& weights,
                             std::vector<double>& rcf) {
    int rcfLen = static_cast<int>(weights.size());
    rcf.assign(rcfLen, 0.0);
    
    const int numElements = 4;  // Comb filter elements
    
    for (int i = 2; i < rcfLen; ++i) {
        for (int a = 1; a <= numElements; ++a) {
            for (int b = 1 - a; b <= a - 1; ++b) {
                int idx = a * i + b - 1;
                if (idx >= 0 && idx < static_cast<int>(acf.size())) {
                    rcf[i - 1] += (acf[idx] * weights[i - 1]) / (2.0 * a - 1.0);
                }
            }
        }
    }
    
    adaptiveThreshold(rcf);
    
    // Normalisieren
    const double eps = 1e-8;
    double sum = std::accumulate(rcf.begin(), rcf.end(), 0.0) + eps;
    for (double& val : rcf) {
        val = (val + eps) / sum;
    }
}

void TempoTracker::viterbiDecode(const std::vector<std::vector<double>>& rcfMatrix,
                                const std::vector<double>& weights,
                                std::vector<int>& beatPeriod) {
    if (rcfMatrix.size() < 2) return;
    
    const size_t T = rcfMatrix.size();      // Zeitschritte
    const size_t Q = rcfMatrix[0].size();   // Zustände (Perioden)
    
    // Transitions-Matrix (Gaussian um Diagonale)
    // sigma=14 erlaubt dem Viterbi-Decoder flexiblere Tempo-Wechsel
    std::vector<std::vector<double>> tmat(Q, std::vector<double>(Q, 0.0));
    const double sigma = 14.0;
    
    for (size_t i = 20; i < Q - 20; ++i) {
        for (size_t j = 20; j < Q - 20; ++j) {
            double mu = static_cast<double>(i);
            double diff = static_cast<double>(j) - mu;
            tmat[i][j] = std::exp(-diff * diff / (2.0 * sigma * sigma));
        }
    }
    
    // Viterbi Variablen
    std::vector<std::vector<double>> delta(T, std::vector<double>(Q, 0.0));
    std::vector<std::vector<int>> psi(T, std::vector<int>(Q, 0));
    
    // Initialisierung
    const double eps = 1e-8;
    double deltaSum = 0.0;
    for (size_t j = 0; j < Q; ++j) {
        delta[0][j] = weights[j] * rcfMatrix[0][j];
        deltaSum += delta[0][j];
    }
    for (size_t j = 0; j < Q; ++j) {
        delta[0][j] /= (deltaSum + eps);
    }
    
    // Rekursion
    for (size_t t = 1; t < T; ++t) {
        for (size_t j = 0; j < Q; ++j) {
            double maxVal = 0.0;
            int maxIdx = 0;
            
            for (size_t i = 0; i < Q; ++i) {
                double val = delta[t - 1][i] * tmat[j][i];
                if (val > maxVal) {
                    maxVal = val;
                    maxIdx = static_cast<int>(i);
                }
            }
            
            delta[t][j] = maxVal * rcfMatrix[t][j];
            psi[t][j] = maxIdx;
        }
        
        // Normalisierung
        deltaSum = 0.0;
        for (size_t j = 0; j < Q; ++j) {
            deltaSum += delta[t][j];
        }
        for (size_t j = 0; j < Q; ++j) {
            delta[t][j] /= (deltaSum + eps);
        }
    }
    
    // Backtracking
    beatPeriod.resize(T);
    
    // Finde besten Endzustand
    double maxVal = 0.0;
    int maxIdx = 0;
    for (size_t j = 0; j < Q; ++j) {
        if (delta[T - 1][j] > maxVal) {
            maxVal = delta[T - 1][j];
            maxIdx = static_cast<int>(j);
        }
    }
    
    beatPeriod[T - 1] = maxIdx;
    for (int t = static_cast<int>(T) - 2; t >= 0; --t) {
        beatPeriod[t] = psi[t + 1][beatPeriod[t + 1]];
    }
}

void TempoTracker::calculateBeatPeriod(const std::vector<double>& df,
                                       std::vector<int>& beatPeriod,
                                       double inputTempo) {
    // Weight vector muss alle möglichen Beat-Perioden abdecken
    // Bei hop=256, sr=44100: 60 BPM = 60/60*44100/256 = 172 Frames
    // Sicherheitsmarge: 256 Frames
    const int wvLen = 256;  // Weight vector length
    
    // Rayleigh Parameter basierend auf Tempo
    // 60 * sampleRate / hopSize ist magische Zahl für BPM-zu-Period Konversion
    double rayparam = (60.0 * m_sampleRate / m_hopSize) / inputTempo;
    
    // Rayleigh Gewichtungs-Kurve mit flacherem Floor
    // Uniform Floor verhindert dass falsche inputTempo andere BPM-Werte komplett ausblendet
    std::vector<double> weights(wvLen);
    double maxWeight = 0.0;
    for (int i = 0; i < wvLen; ++i) {
        double x = static_cast<double>(i);
        weights[i] = (x / (rayparam * rayparam)) * 
                     std::exp(-x * x / (2.0 * rayparam * rayparam));
        if (weights[i] > maxWeight) maxWeight = weights[i];
    }
    // Addiere 20% uniform floor damit auch entferntere Tempi eine Chance haben
    double uniformFloor = maxWeight * 0.2;
    for (int i = 0; i < wvLen; ++i) {
        weights[i] += uniformFloor;
    }
    
    // Analyse-Fenster (muss > 2x max Beat-Periode sein für gute ACF)
    const int winLen = 1024;
    const int hopSize = 256;
    
    int dfLen = static_cast<int>(df.size());
    
    // Filterung
    std::vector<double> filteredDf = df;
    filterDF(filteredDf);
    
    // RCF Matrix aufbauen
    std::vector<std::vector<double>> rcfMatrix;
    std::vector<double> dfFrame(winLen, 0.0);
    std::vector<double> acf(winLen);
    std::vector<double> rcf(wvLen);
    
    for (int i = -winLen / 2; i < dfLen - winLen / 2; i += hopSize) {
        // Frame extrahieren mit Zero-Padding
        std::fill(dfFrame.begin(), dfFrame.end(), 0.0);
        
        int start = std::max(0, i);
        int end = std::min(dfLen, i + winLen);
        int offset = (i < 0) ? -i : 0;
        
        for (int j = start; j < end; ++j) {
            dfFrame[offset + j - start] = filteredDf[j];
        }
        
        // Autocorrelation
        autocorrelate(dfFrame, acf);
        
        // Comb Filter
        combFilter(acf, weights, rcf);
        
        rcfMatrix.push_back(rcf);
    }
    
    // Viterbi Decoding
    viterbiDecode(rcfMatrix, weights, beatPeriod);
}

void TempoTracker::calculateBeats(const std::vector<double>& df,
                                 const std::vector<int>& beatPeriod,
                                 std::vector<double>& beats) {
    beats.clear();
    
    if (df.empty() || beatPeriod.empty()) return;
    
    int dfLen = static_cast<int>(df.size());
    
    // Berechne durchschnittliche Periode
    double avgPeriod = 0.0;
    for (int p : beatPeriod) {
        avgPeriod += p;
    }
    avgPeriod /= beatPeriod.size();
    
    if (avgPeriod < 2) return;
    
    // Finde Peaks in der Detection Function
    std::vector<int> peaks;
    for (int i = 1; i < dfLen - 1; ++i) {
        if (df[i] > df[i - 1] && df[i] > df[i + 1]) {
            peaks.push_back(i);
        }
    }
    
    if (peaks.empty()) return;
    
    // Dynamic Programming für optimale Beat-Sequenz
    // Vereinfachte Version: Nutze Peaks mit konsistentem Abstand
    double beatInterval = avgPeriod;
    double currentBeat = 0.0;
    
    // Finde ersten starken Peak
    double maxVal = 0.0;
    int firstPeak = 0;
    for (int i = 0; i < std::min(static_cast<int>(peaks.size()), 10); ++i) {
        if (df[peaks[i]] > maxVal) {
            maxVal = df[peaks[i]];
            firstPeak = peaks[i];
        }
    }
    
    // Generiere Beats basierend auf Interval
    currentBeat = firstPeak;
    while (currentBeat < dfLen) {
        beats.push_back(currentBeat);
        currentBeat += beatInterval;
    }
    
    // Rückwärts auch
    currentBeat = firstPeak - beatInterval;
    while (currentBeat >= 0) {
        beats.insert(beats.begin(), currentBeat);
        currentBeat -= beatInterval;
    }
}

// ============================================================================
// Real-Time Beat Tracker
// ============================================================================

RealTimeBeatTracker::RealTimeBeatTracker(const BeatDetectorConfig& config)
    : m_config(config),
      m_bufferWritePos(0),
      m_currentBpm(0.0),
      m_totalFramesProcessed(0) {
}

bool RealTimeBeatTracker::initialize() {
    m_onsetDetector = std::make_unique<OnsetDetector>(m_config);
    m_tempoTracker = std::make_unique<TempoTracker>(m_config.sampleRate, m_config.hopSize);
    m_beatEventDetector = std::make_unique<BeatEventDetector>(
        m_config.sampleRate,
        m_config.hopSize,
        m_config.maxBpm);
    
    m_monoBuffer.resize(m_config.frameSize, 0.0);
    m_frameBuffer.resize(m_config.frameSize, 0.0);
    m_detectionFunction.clear();
    m_bufferWritePos = 0;
    
    return true;
}

void RealTimeBeatTracker::downmixToMono(const Sample* stereo, 
                                        double* mono, int frames) {
    for (int i = 0; i < frames; ++i) {
        mono[i] = (static_cast<double>(stereo[i * 2]) + 
                   static_cast<double>(stereo[i * 2 + 1])) * 0.5;
    }
}

void RealTimeBeatTracker::processAudio(const Sample* stereoInput, int frameCount) {
    m_beatOccurred = false;  // Reset am Anfang jedes Blocks
    std::vector<double> monoInput(frameCount);
    downmixToMono(stereoInput, monoInput.data(), frameCount);
    
    processMonoInternal(monoInput.data(), frameCount);
}

void RealTimeBeatTracker::processMonoAudio(const Sample* monoInput, int frameCount) {
    m_beatOccurred = false;  // Reset am Anfang jedes Blocks
    
    // Float zu Double konvertieren MIT Auto Gain Control
    std::vector<double> monoDouble(frameCount);
    
    if (m_config.agcEnabled) {
        // Peak-Tracking: schneller Attack, langsamer Release
        float peak = 0.0f;
        for (int i = 0; i < frameCount; ++i) {
            float absVal = std::fabs(monoInput[i]);
            if (absVal > peak) peak = absVal;
        }
        
        // Peak-Tracker aktualisieren
        if (peak > m_agcPeakTracker) {
            m_agcPeakTracker = m_agcPeakTracker * 0.3f + peak * 0.7f;  // Schneller Attack
        } else {
            m_agcPeakTracker = m_agcPeakTracker * 0.999f + peak * 0.001f;  // Sehr langsamer Release
        }
        
        // Gain berechnen um Ziel-Peak zu erreichen
        if (m_agcPeakTracker > 1e-6f) {
            float desiredGain = m_config.agcTargetPeak / m_agcPeakTracker;
            // Gain begrenzen: min 1x (nie leiser), max 100x (+40dB)
            desiredGain = std::max(1.0f, std::min(desiredGain, 100.0f));
            // Sanfte Gain-Änderung
            m_agcGain = m_agcGain * 0.95f + desiredGain * 0.05f;
        }
        
        for (int i = 0; i < frameCount; ++i) {
            monoDouble[i] = static_cast<double>(monoInput[i]) * m_agcGain;
        }
    } else {
        for (int i = 0; i < frameCount; ++i) {
            monoDouble[i] = static_cast<double>(monoInput[i]);
        }
    }
    
    processMonoInternal(monoDouble.data(), frameCount);
}

void RealTimeBeatTracker::processMonoInternal(const double* monoInput, int frameCount) {
    for (int i = 0; i < frameCount; ++i) {
        // Sicherstellen dass wir nicht über den Buffer hinaus schreiben
        if (m_bufferWritePos >= m_monoBuffer.size()) {
            m_bufferWritePos = 0;
        }
        
        m_monoBuffer[m_bufferWritePos] = monoInput[i];
        m_bufferWritePos++;
        m_totalFramesProcessed++;
        
        // Wenn wir genug Samples haben
        if (m_bufferWritePos >= static_cast<size_t>(m_config.frameSize)) {
            // Kopiere Frame für Analyse
            std::copy(m_monoBuffer.begin(), 
                     m_monoBuffer.begin() + m_config.frameSize,
                     m_frameBuffer.begin());
            
            // Onset Detection
            double onset = m_onsetDetector->process(m_frameBuffer.data());
            
            // Tempo-Gating: wenn BPM bekannt, Onsets nahe erwartetem Beat leicht boosten
            if (m_currentBpm > 0 && m_lastBeatOnsetFrame > 0) {
                int64_t currentDfFrame = static_cast<int64_t>(m_detectionFunction.size());
                double beatsPerDfFrame = m_currentBpm / 60.0 * m_config.hopSize / m_config.sampleRate;
                double dfFramesPerBeat = 1.0 / beatsPerDfFrame;
                int64_t frameSinceBeat = currentDfFrame - m_lastBeatOnsetFrame;
                
                // Nächster erwarteter Beat
                double phaseInBeat = static_cast<double>(frameSinceBeat) / dfFramesPerBeat;
                phaseInBeat -= std::floor(phaseInBeat);  // 0..1
                
                // Fenster um erwarteten Beat: +-10% des Beat-Intervalls
                if (phaseInBeat > 0.9 || phaseInBeat < 0.1) {
                    onset *= 1.3;  // 30% Boost nahe erwartetem Beat
                } else if (phaseInBeat > 0.3 && phaseInBeat < 0.7) {
                    onset *= 0.85;  // 15% Dämpfung weit vom erwarteten Beat
                }
            }
            
            m_detectionFunction.push_back(onset);
            
            // Detection Function begrenzen (max ~16 Sekunden)
            const size_t maxDfSize = static_cast<size_t>(16.0 * m_config.sampleRate / m_config.hopSize);
            if (m_detectionFunction.size() > maxDfSize) {
                // Anzahl zu entfernender Frames
                size_t toRemove = m_detectionFunction.size() - maxDfSize;
                m_detectionFunction.erase(m_detectionFunction.begin(), 
                    m_detectionFunction.begin() + toRemove);
                // lastBeatOnsetFrame korrigieren
                m_lastBeatOnsetFrame -= static_cast<int64_t>(toRemove);
                if (m_lastBeatOnsetFrame < 0) m_lastBeatOnsetFrame = 0;
            }
            
            // Beat Event Detection
            if (m_beatEventDetector->process(onset)) {
                m_beatOccurred = true;
                m_lastBeatOnsetFrame = static_cast<int64_t>(m_detectionFunction.size()) - 1;
            }
            
            // Echtzeit BPM-Schätzung alle ~0.35 Sekunden
            // Bei hop=256, sr=44100: 0.35s ≈ 60 Frames
            size_t updateInterval = static_cast<size_t>(0.35 * m_config.sampleRate / m_config.hopSize);
            if (updateInterval < 10) updateInterval = 10;
            if (m_detectionFunction.size() >= 100 && 
                m_detectionFunction.size() % updateInterval == 0) {
                updateRealtimeBpm();
            }
            
            // Shift buffer um hopSize
            std::copy(m_monoBuffer.begin() + m_config.hopSize,
                     m_monoBuffer.end(),
                     m_monoBuffer.begin());
            m_bufferWritePos = m_config.frameSize - m_config.hopSize;
        }
    }
}

void RealTimeBeatTracker::updateRealtimeBpm() {
    if (m_detectionFunction.size() < 50) return;
    
    // Nutze die letzten 10 Sekunden für BPM-Schätzung
    // Bei hop=256, sr=44100: 10s = 10*44100/256 = 1723 Frames
    size_t framesFor10s = static_cast<size_t>(10.0 * m_config.sampleRate / m_config.hopSize);
    size_t windowSize = std::min(m_detectionFunction.size(), framesFor10s);
    std::vector<double> df(m_detectionFunction.end() - windowSize, 
                          m_detectionFunction.end());
    
    std::vector<int> beatPeriod;
    // WICHTIG: Nutze defaultBpm als Referenz statt currentBpm um Feedback-Bias zu vermeiden.
    // Der Rayleigh-Gewichtung in calculateBeatPeriod zieht den Schätzer Richtung inputTempo,
    // wenn wir hier den aktuellen Wert einspeisen, verstärkt sich ein falscher BPM-Wert.
    // Nur den Mittelwert des konfigurierten Bereichs verwenden.
    double referenceBpm = (m_config.minBpm + m_config.maxBpm) / 2.0;
    m_tempoTracker->calculateBeatPeriod(df, beatPeriod, referenceBpm);
    
    if (!beatPeriod.empty()) {
        // Median statt Durchschnitt (robuster gegen Ausreißer)
        std::vector<int> sortedPeriods = beatPeriod;
        std::sort(sortedPeriods.begin(), sortedPeriods.end());
        
        // Trimmed median: ignoriere oberes und unteres 10%
        size_t trimCount = sortedPeriods.size() / 10;
        double sumPeriod = 0;
        int validCount = 0;
        for (size_t i = trimCount; i < sortedPeriods.size() - trimCount; ++i) {
            sumPeriod += sortedPeriods[i];
            validCount++;
        }
        double medianPeriod = (validCount > 0) ? (sumPeriod / validCount) : sortedPeriods[sortedPeriods.size() / 2];
        
        if (medianPeriod > 0) {
            double newBpm = 60.0 * m_config.sampleRate / (medianPeriod * m_config.hopSize);
            
            // Sanity check: BPM sollte im erlaubten Bereich sein
            if (newBpm >= m_config.minBpm && newBpm <= m_config.maxBpm) {
                if (m_currentBpm > 0) {
                    // Leichtere Glättung: 0.6/0.4 bei großer Änderung, 0.85/0.15 sonst
                    double diff = std::abs(newBpm - m_currentBpm) / m_currentBpm;
                    if (diff > 0.08) {
                        // Große Änderung: schnell reagieren
                        m_currentBpm = m_currentBpm * 0.4 + newBpm * 0.6;
                    } else {
                        // Fein-Stabilisierung
                        m_currentBpm = m_currentBpm * 0.85 + newBpm * 0.15;
                    }
                } else {
                    m_currentBpm = newBpm;
                }
            }
        }
    }
}

BeatInfo RealTimeBeatTracker::finalize() {
    BeatInfo result;
    
    if (m_detectionFunction.size() < 10) {
        return result;
    }
    
    // Skip erste 2 Werte (können Rauschen enthalten)
    std::vector<double> df(m_detectionFunction.begin() + 2, 
                          m_detectionFunction.end());
    
    // Beat Period berechnen
    std::vector<int> beatPeriod;
    m_tempoTracker->calculateBeatPeriod(df, beatPeriod, m_config.defaultBpm);
    
    // Beats berechnen
    std::vector<double> beats;
    m_tempoTracker->calculateBeats(df, beatPeriod, beats);
    
    // Konvertiere zu Frame-Positionen
    for (double beat : beats) {
        // +2 wegen übersprungener Anfangswerte
        int64_t framePos = static_cast<int64_t>((beat + 2) * m_config.hopSize) 
                         + m_config.hopSize / 2;
        result.beatFrames.push_back(framePos);
    }
    
    // BPM berechnen
    if (!beatPeriod.empty()) {
        double avgPeriod = 0.0;
        for (int p : beatPeriod) {
            avgPeriod += p;
        }
        avgPeriod /= beatPeriod.size();
        
        if (avgPeriod > 0) {
            result.bpm = 60.0 * m_config.sampleRate / (avgPeriod * m_config.hopSize);
            m_currentBpm = result.bpm;
        }
    }
    
    result.confidence = 0.9;  // TODO: Berechne echte Konfidenz
    result.valid = result.bpm > 0 && !result.beatFrames.empty();
    
    return result;
}

void RealTimeBeatTracker::reset() {
    m_onsetDetector->reset();
    std::fill(m_monoBuffer.begin(), m_monoBuffer.end(), 0.0);
    m_detectionFunction.clear();
    m_bufferWritePos = 0;
    m_totalFramesProcessed = 0;
    m_currentBpm = 0.0;
}


// ============================================================================
// BeatEventDetector Implementation
// ============================================================================

BeatEventDetector::BeatEventDetector(int sampleRate, int hopSize, double maxBpm)
    : m_sampleRate(sampleRate) {
    // Minimum-Abstand zwischen Beats basierend auf max BPM
    // minBeatSec = 60/maxBpm, mit 15% Toleranz für frühe Peaks
    double minBeatSec = (60.0 / maxBpm) * 0.85;
    m_minBeatInterval = static_cast<int>(minBeatSec * sampleRate / hopSize);
    if (m_minBeatInterval < 6) m_minBeatInterval = 6;  // Minimum ~70ms bei hop=512
    reset();
}

bool BeatEventDetector::process(double onsetValue) {
    m_framesSinceLastBeat++;
    
    // Laufenden Mittelwert für Normalisierung aktualisieren (schneller: α=0.08)
    const double alpha = 0.08;
    m_runningMean = m_runningMean * (1.0 - alpha) + onsetValue * alpha;
    double deviation = std::abs(onsetValue - m_runningMean);
    m_runningDev = m_runningDev * (1.0 - alpha) + deviation * alpha;
    
    // Adaptive Threshold aktualisieren (langsam abklingend)
    if (onsetValue > m_adaptiveThreshold) {
        m_adaptiveThreshold = onsetValue;
    } else {
        m_adaptiveThreshold *= THRESHOLD_DECAY;
    }
    
    // Early exit bei sehr kleinem Signal
    if (m_runningMean < 1e-6 && onsetValue < 1e-6) {
        m_prevPrevOnset = m_prevOnset;
        m_prevOnset = onsetValue;
        return false;
    }
    
    // Threshold: Kombination aus adaptivem Maximum und dynamischer Streuung
    // mean + 1.8*dev ist empfindlicher aber immer noch stabil
    double dynamicThreshold = m_runningMean + (m_runningDev * 1.8);
    m_threshold = std::max(m_adaptiveThreshold * 0.35, dynamicThreshold);
    
    // Peak Detection: prevOnset muss größer sein als Nachbarn UND über Threshold
    bool isPeak = (m_prevOnset > m_prevPrevOnset) && 
                  (m_prevOnset > onsetValue) &&
                  (m_prevOnset > m_threshold);
    
    // Minimum Interval zwischen Beats
    bool beatDetected = isPeak && (m_framesSinceLastBeat >= m_minBeatInterval);
    
    if (beatDetected) {
        m_framesSinceLastBeat = 0;
    }
    
    // History aktualisieren
    m_prevPrevOnset = m_prevOnset;
    m_prevOnset = onsetValue;
    
    return beatDetected;
}

void BeatEventDetector::reset() {
    m_threshold = 0.0;
    m_adaptiveThreshold = 0.0;
    m_runningMean = 0.0;
    m_runningDev = 0.0;
    m_prevOnset = 0.0;
    m_prevPrevOnset = 0.0;
    m_framesSinceLastBeat = m_minBeatInterval;  // Erlaube sofort Beat
}

} // namespace BeatAnalyzer
