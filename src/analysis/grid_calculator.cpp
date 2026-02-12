#include "analysis/grid_calculator.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace BeatAnalyzer {

GridCalculator::GridCalculator(int sampleRate, double windowSeconds, double minBpm, double maxBpm)
    : m_sampleRate(sampleRate)
    , m_windowSeconds(windowSeconds)
    , m_minBpm(minBpm)
    , m_maxBpm(maxBpm)
    , m_windowFrames(static_cast<int64_t>(windowSeconds * sampleRate))
{
}

bool GridCalculator::addRealbeat(int64_t frame, int64_t currentFrame, float strength) {
    // Füge neuen Beat hinzu (immer speichern für Analyse)
    m_realbeatFrames.push_back(frame);
    m_realbeatStrengths.push_back(strength);
    
    // Entferne alte Beats außerhalb des Fensters
    int64_t windowStart = currentFrame - m_windowFrames;
    while (!m_realbeatFrames.empty() && m_realbeatFrames.front() < windowStart) {
        m_realbeatFrames.pop_front();
        if (!m_realbeatStrengths.empty()) m_realbeatStrengths.pop_front();
    }
    
    // Prüfe ob REALBEAT zur Maske passt (TAP oder Auto)
    bool matches = matchesMask(frame);
    
    if (matches) {
        // Füge zu maskierten REALBEATs hinzu
        m_maskedRealbeatFrames.push_back(frame);
        
        // Entferne alte maskierte Beats
        while (!m_maskedRealbeatFrames.empty() && m_maskedRealbeatFrames.front() < windowStart) {
            m_maskedRealbeatFrames.pop_front();
        }
        
        // === MASKE KONTINUIERLICH KORRIGIEREN ===
        // Wenn genug maskierte REALBEATs vorhanden, Maske anpassen
        if (m_maskSlotsFilled >= 2 && m_maskedRealbeatFrames.size() >= 4) {
            adjustMaskFromRealbeats();
        }
    }
    // Auch bei Nicht-Treffer: Prüfen ob Maske komplett daneben liegt
    else if (m_maskSlotsFilled >= 2 && m_maskIntervalFrames > 0) {
        // Zähle konsekutive Misses
        m_consecutiveMisses++;
        
        // Nach 8 Misses: Maske neu berechnen aus allen REALBEATs
        if (m_consecutiveMisses >= 8 && m_realbeatFrames.size() >= 8) {
            recalculateMaskFromRealbeats();
            m_consecutiveMisses = 0;
        }
    }
    
    return matches;
}

bool GridCalculator::matchesMask(int64_t realbeatFrame) const {
    // Prüfe TAP-Maske zuerst
    if (m_tapFrames.size() >= 2 && m_tapBpm > 0.0) {
        return matchesTapMask(realbeatFrame);
    }
    
    // Prüfe Auto-Maske
    if (m_autoMaskActive && m_autoMaskBpm > 0.0) {
        double framesPerBeat = 60.0 * m_sampleRate / m_autoMaskBpm;
        double toleranceFrames = framesPerBeat * 0.15;  // ±15% Toleranz
        
        double offset = static_cast<double>(realbeatFrame - m_autoMaskFirstBeat);
        double beatNumber = offset / framesPerBeat;
        double roundedBeat = std::round(beatNumber);
        int64_t expectedFrame = m_autoMaskFirstBeat + static_cast<int64_t>(roundedBeat * framesPerBeat);
        
        double error = std::abs(static_cast<double>(realbeatFrame - expectedFrame));
        return error < toleranceFrames;
    }
    
    // Keine Maske → alle akzeptieren
    return true;
}

bool GridCalculator::matchesTapMask(int64_t realbeatFrame) const {
    // Ohne Spalte definiert → alle REALBEATs akzeptieren
    if (m_maskSlotsFilled < 1) {
        return true;
    }
    
    // Nur Spalt 1 definiert (erster TAP) → akzeptiere alles
    // Wir warten auf den zweiten TAP um das Intervall zu kennen
    if (m_maskSlotsFilled == 1) {
        return true;
    }
    
    // Ab 2+ Spalte: Prüfe ob REALBEAT zu einem Spalt passt
    double toleranceFrames = m_maskIntervalFrames * m_maskTolerancePercent;
    
    // 16-Beat Periode (4 Takte)
    double periodFrames = m_maskSlotCount * m_maskIntervalFrames;
    
    // Finde nächsten erwarteten Slot relativ zum ersten Slot
    if (m_maskIntervalFrames > 0 && m_maskSlots[0] != 0) {
        double offset = static_cast<double>(realbeatFrame - m_maskSlots[0]);
        
        // Normalisiere auf 16-Beat-Periode
        double periods = offset / periodFrames;
        double fractionalOffset = (periods - std::floor(periods)) * periodFrames;
        if (fractionalOffset < 0) fractionalOffset += periodFrames;
        
        // Finde nächsten Spalt in dieser Periode
        for (int j = 0; j < m_maskSlotCount; ++j) {
            double slotOffset = j * m_maskIntervalFrames;
            double error = std::abs(fractionalOffset - slotOffset);
            // Auch wrap-around prüfen
            double errorWrap = std::abs(fractionalOffset - periodFrames - slotOffset);
            error = std::min(error, errorWrap);
            
            if (error < toleranceFrames) {
                return true;
            }
        }
    }
    
    return false;
}

void GridCalculator::addTap(int64_t frame, int64_t currentFrame, bool isFirstTap) {
    // TAP überschreibt Auto-Maske
    m_autoMaskActive = false;
    m_autoMaskFirstBeat = 0;
    m_autoMaskBpm = 0.0;
    
    // Erster TAP nach Reset → Spalt 1 setzen
    if (isFirstTap || m_tapFrames.empty()) {
        m_firstTapFrame = frame;
        m_tapBpm = 0.0;
        m_maskedRealbeatFrames.clear();
        
        // Maske zurücksetzen: Spalt 1 auf ersten TAP
        for (int i = 0; i < m_maskSlotCount; ++i) {
            m_maskSlots[i] = 0;
        }
        m_maskSlots[0] = frame;
        m_maskSlotsFilled = 1;
        m_maskIntervalFrames = 0.0;
        m_maskTolerancePercent = 0.12;  // Starte mit ±12% Toleranz
    }
    // Zweiter TAP → Spalt 2 setzen, Intervall berechnen, alle 16 Spalte vorhersagen
    else if (m_maskSlotsFilled == 1) {
        m_maskSlots[1] = frame;
        m_maskIntervalFrames = static_cast<double>(frame - m_maskSlots[0]);
        
        // Alle 16 Spalte vorhersagen (gleichmäßig verteilt)
        for (int i = 2; i < m_maskSlotCount; ++i) {
            m_maskSlots[i] = m_maskSlots[0] + static_cast<int64_t>(i * m_maskIntervalFrames);
        }
        m_maskSlotsFilled = m_maskSlotCount;  // Alle 16 Spalte definiert
        
        // BPM aus Intervall
        m_tapBpm = 60.0 * m_sampleRate / m_maskIntervalFrames;
    }
    // Weitere TAPs → Maske verfeinern (Spaltbreite anpassen)
    else if (m_maskSlotsFilled >= 2) {
        // 16-Beat Periode (4 Takte)
        double periodFrames = m_maskSlotCount * m_maskIntervalFrames;
        
        // Finde welcher Slot-Index in der aktuellen Periode getroffen wurde
        double offset = static_cast<double>(frame - m_maskSlots[0]);
        double periods = offset / periodFrames;
        double fractionalOffset = (periods - std::floor(periods)) * periodFrames;
        if (fractionalOffset < 0) fractionalOffset += periodFrames;
        
        int bestSlot = static_cast<int>(std::round(fractionalOffset / m_maskIntervalFrames));
        bestSlot = bestSlot % m_maskSlotCount;
        
        double expectedPos = bestSlot * m_maskIntervalFrames;
        double bestError = std::abs(fractionalOffset - expectedPos);
        double toleranceFrames = m_maskIntervalFrames * m_maskTolerancePercent;
        
        if (bestError < toleranceFrames) {
            // Slot gefunden → Intervall sanft korrigieren (DJ-style: kleine Schritte)
            // Berechne ideales Intervall aus diesem TAP
            double idealInterval = (frame - m_maskSlots[0]) / (std::floor(periods) * m_maskSlotCount + bestSlot);
            if (idealInterval > 0 && std::floor(periods) * m_maskSlotCount + bestSlot > 0) {
                // Sanfte Korrektur: nur 5% pro TAP (wie DJ langsam am Pitch dreht)
                double correction = (idealInterval - m_maskIntervalFrames) * 0.05;
                m_maskIntervalFrames += correction;
                m_tapBpm = 60.0 * m_sampleRate / m_maskIntervalFrames;
            }
            
            // Spaltbreite enger machen wenn TAP gut getroffen hat
            if (bestError < toleranceFrames * 0.5) {
                m_maskTolerancePercent = std::max(0.05, m_maskTolerancePercent * 0.9);
            }
            
            // Alle Slots neu berechnen basierend auf korrigiertem Intervall
            for (int i = 0; i < m_maskSlotCount; ++i) {
                m_maskSlots[i] = m_maskSlots[0] + static_cast<int64_t>(i * m_maskIntervalFrames);
            }
        }
    }
    
    // Füge TAP hinzu
    m_tapFrames.push_back(frame);
    
    // Entferne alte TAPs außerhalb des Fensters (längeres Fenster für TAPs: 30s)
    int64_t tapWindowFrames = static_cast<int64_t>(30.0 * m_sampleRate);
    int64_t windowStart = currentFrame - tapWindowFrames;
    while (!m_tapFrames.empty() && m_tapFrames.front() < windowStart) {
        m_tapFrames.pop_front();
    }
}

GridResult GridCalculator::calculateGrid(double tapPatternBpm, int64_t referenceFrame) {
    GridResult result;
    
    // SOLL-GRID aus TAPs berechnen (falls genug TAPs vorhanden)
    if (m_tapFrames.size() >= 2) {
        double tapBpm = calculateTapBpm();
        if (tapBpm >= m_minBpm && tapBpm <= m_maxBpm) {
            result.sollBpm = tapBpm;
            result.sollPhase = calculateTapPhase(tapBpm, referenceFrame);
            result.hasSollGrid = true;
            
            // Überlappung zwischen TAP und REALBEAT prüfen
            double overlap = calculateTapRealbeatOverlap(tapBpm);
            
            // Wenn gute Überlappung → SOLL-GRID ist validiert
            if (overlap > 0.5) {
                result.gridBpm = tapBpm;
                result.gridPhase = result.sollPhase;
                result.confidence = overlap;
                result.valid = true;
                return result;
            }
        }
    }
    
    // Mindestens 3 Beats für REALBEAT-Analyse
    if (m_realbeatFrames.size() < 3) {
        // Wenn SOLL-GRID vorhanden aber keine REALBEATs → SOLL-GRID nutzen
        if (result.hasSollGrid) {
            result.gridBpm = result.sollBpm;
            result.gridPhase = result.sollPhase;
            result.confidence = 0.5;  // Moderate Confidence ohne REALBEAT-Bestätigung
            result.valid = true;
        }
        return result;
    }
    
    double detectedBpm = 0.0;
    
    // TAP-Muster als Suchvorgabe nutzen
    double searchBpm = (result.hasSollGrid) ? result.sollBpm : tapPatternBpm;
    
    if (searchBpm > 0.0) {
        // TAP-Muster vorgegeben: Suche nach passenden Intervallen
        detectedBpm = calculateIntervalHistogramPeak(searchBpm);
        
        // Wenn kein passendes Muster gefunden, fallback auf freie Erkennung
        if (detectedBpm <= 0.0) {
            detectedBpm = calculateIntervalHistogramPeak(0.0);
        }
    } else {
        // Freie Erkennung ohne Muster
        detectedBpm = calculateIntervalHistogramPeak(0.0);
    }
    
    // Validierung
    if (detectedBpm < m_minBpm || detectedBpm > m_maxBpm) {
        return result;
    }
    
    result.gridBpm = detectedBpm;
    result.gridPhase = calculatePhase(detectedBpm, referenceFrame);
    result.confidence = calculateConfidence(detectedBpm);
    result.valid = (result.confidence > 0.3);  // Mindest-Confidence für Gültigkeit
    
    return result;
}

void GridCalculator::reset() {
    m_realbeatFrames.clear();
    m_realbeatStrengths.clear();
    m_maskedRealbeatFrames.clear();
    m_tapFrames.clear();
    m_firstTapFrame = 0;
    m_tapBpm = 0.0;
    m_autoMaskActive = false;
    m_autoMaskFirstBeat = 0;
    m_autoMaskBpm = 0.0;
    
    // Adaptive Maske zurücksetzen
    for (int i = 0; i < m_maskSlotCount; ++i) {
        m_maskSlots[i] = 0;
    }
    m_maskSlotsFilled = 0;
    m_maskIntervalFrames = 0.0;
    m_maskTolerancePercent = 0.12;
    m_consecutiveMisses = 0;
}

void GridCalculator::adjustMaskFromRealbeats() {
    // Passe Maske basierend auf maskierten REALBEATs an
    if (m_maskedRealbeatFrames.size() < 4 || m_maskIntervalFrames <= 0) {
        return;
    }
    
    // Berechne durchschnittliches Intervall aus maskierten REALBEATs
    std::vector<double> intervals;
    for (size_t i = 1; i < m_maskedRealbeatFrames.size(); ++i) {
        double interval = static_cast<double>(m_maskedRealbeatFrames[i] - m_maskedRealbeatFrames[i-1]);
        // Nur Intervalle im erwarteten Bereich (0.5x bis 2x des erwarteten)
        if (interval > m_maskIntervalFrames * 0.5 && interval < m_maskIntervalFrames * 2.0) {
            // Normalisiere auf 1-Beat-Intervall
            double beats = std::round(interval / m_maskIntervalFrames);
            if (beats > 0) {
                intervals.push_back(interval / beats);
            }
        }
    }
    
    if (intervals.size() < 3) {
        return;
    }
    
    // Median-Intervall berechnen
    std::sort(intervals.begin(), intervals.end());
    double medianInterval = intervals[intervals.size() / 2];
    
    // Sanfte Korrektur des Masken-Intervalls (10% pro Update)
    double correction = (medianInterval - m_maskIntervalFrames) * 0.1;
    m_maskIntervalFrames += correction;
    
    // BPM aktualisieren
    m_tapBpm = 60.0 * m_sampleRate / m_maskIntervalFrames;
    
    // Spalte neu berechnen basierend auf letztem maskierten REALBEAT
    int64_t lastMasked = m_maskedRealbeatFrames.back();
    
    // Berechne Phase-Offset des letzten maskierten REALBEATs
    double periodFrames = m_maskSlotCount * m_maskIntervalFrames;
    double offset = static_cast<double>(lastMasked - m_maskSlots[0]);
    double periods = offset / periodFrames;
    double fractionalOffset = (periods - std::floor(periods)) * periodFrames;
    if (fractionalOffset < 0) fractionalOffset += periodFrames;
    
    int closestSlot = static_cast<int>(std::round(fractionalOffset / m_maskIntervalFrames));
    closestSlot = closestSlot % m_maskSlotCount;
    
    double expectedPos = closestSlot * m_maskIntervalFrames;
    double phaseError = fractionalOffset - expectedPos;
    
    // Sanfte Phase-Korrektur (DJ-style: 3% pro Update)
    double phaseCorrection = phaseError * 0.03;
    m_maskSlots[0] += static_cast<int64_t>(phaseCorrection);
    
    // Alle Slots neu berechnen
    for (int i = 1; i < m_maskSlotCount; ++i) {
        m_maskSlots[i] = m_maskSlots[0] + static_cast<int64_t>(i * m_maskIntervalFrames);
    }
    
    // Miss-Counter zurücksetzen bei erfolgreicher Anpassung
    m_consecutiveMisses = 0;
}

void GridCalculator::recalculateMaskFromRealbeats() {
    // Komplette Neuberechnung der Maske aus allen REALBEATs
    if (m_realbeatFrames.size() < 8) {
        return;
    }
    
    // Berechne BPM aus Intervall-Histogramm
    double detectedBpm = calculateIntervalHistogramPeak(0.0);
    if (detectedBpm < m_minBpm || detectedBpm > m_maxBpm) {
        return;
    }
    
    double newInterval = 60.0 * m_sampleRate / detectedBpm;
    
    // Finde besten Startpunkt (Phase) für die neue Maske
    // Teste verschiedene Phasen und zähle Treffer
    int bestPhaseHits = 0;
    int64_t bestFirstBeat = m_realbeatFrames.back();
    
    // Teste die letzten 4 REALBEATs als mögliche Beat-1-Positionen
    size_t startIdx = m_realbeatFrames.size() > 4 ? m_realbeatFrames.size() - 4 : 0;
    for (size_t i = startIdx; i < m_realbeatFrames.size(); ++i) {
        int64_t testFirstBeat = m_realbeatFrames[i];
        int hits = 0;
        
        // Zähle wie viele REALBEATs zu diesem Grid passen
        for (const auto& beat : m_realbeatFrames) {
            double offset = static_cast<double>(beat - testFirstBeat);
            double beatNum = offset / newInterval;
            double error = std::abs(beatNum - std::round(beatNum)) * newInterval;
            if (error < newInterval * 0.15) {
                hits++;
            }
        }
        
        if (hits > bestPhaseHits) {
            bestPhaseHits = hits;
            bestFirstBeat = testFirstBeat;
        }
    }
    
    // Neue Maske setzen mit allen 16 Slots
    m_maskIntervalFrames = newInterval;
    m_tapBpm = detectedBpm;
    m_maskSlots[0] = bestFirstBeat;
    for (int i = 1; i < m_maskSlotCount; ++i) {
        m_maskSlots[i] = bestFirstBeat + static_cast<int64_t>(i * newInterval);
    }
    m_maskSlotsFilled = m_maskSlotCount;
    m_maskTolerancePercent = 0.12;  // Zurück auf 12% Toleranz
    
    // Maskierte REALBEATs neu filtern
    m_maskedRealbeatFrames.clear();
    for (const auto& beat : m_realbeatFrames) {
        if (matchesTapMask(beat)) {
            m_maskedRealbeatFrames.push_back(beat);
        }
    }
}

bool GridCalculator::calculateAutoMask(int64_t currentFrame, int sampleRate) {
    // Brauchen mindestens 8 REALBEATs für stabile Auto-Maske
    if (m_realbeatFrames.size() < 8 || m_realbeatStrengths.size() < 8) {
        return false;
    }
    
    // === 1. BPM aus Intervall-Histogramm bestimmen ===
    double detectedBpm = calculateIntervalHistogramPeak(0.0);
    if (detectedBpm < m_minBpm || detectedBpm > m_maxBpm) {
        return false;
    }
    
    double framesPerBeat = 60.0 * sampleRate / detectedBpm;
    
    // === 2. Finde die 4 lautesten Beats in einem 4-Beat-Fenster ===
    // Wir suchen das 4-Beat-Fenster mit der höchsten Summe der lautesten Beats
    
    // Gruppiere Beats in 4-Beat-Zyklen basierend auf erkanntem BPM
    // Finde den Startpunkt, bei dem die lautesten Beats auf Position 1 fallen
    
    double bestPhaseScore = 0.0;
    int64_t bestFirstBeat = 0;
    
    // Teste verschiedene Startpunkte (die letzten 4 Beats)
    size_t numBeats = m_realbeatFrames.size();
    for (size_t startIdx = 0; startIdx < std::min(numBeats, size_t(8)); ++startIdx) {
        int64_t candidateFirstBeat = m_realbeatFrames[numBeats - 1 - startIdx];
        
        // Berechne Score: Summe der Stärken von Beats die auf Position 1 fallen
        double score = 0.0;
        int matchCount = 0;
        
        for (size_t i = 0; i < numBeats; ++i) {
            int64_t beatFrame = m_realbeatFrames[i];
            float strength = m_realbeatStrengths[i];
            
            // Welche Position im 4-Beat-Zyklus?
            double offset = static_cast<double>(beatFrame - candidateFirstBeat);
            double beatNumber = offset / framesPerBeat;
            int position = static_cast<int>(std::round(beatNumber)) % 4;
            if (position < 0) position += 4;
            
            // Position 0 (Beat 1) soll die lautesten Beats haben
            // Position 2 (Beat 3) soll die zweitlautesten haben (typisch 4/4)
            if (position == 0) {
                score += strength * 1.0;  // Beat 1 - höchstes Gewicht
                matchCount++;
            } else if (position == 2) {
                score += strength * 0.5;  // Beat 3 - mittleres Gewicht
            }
        }
        
        if (matchCount > 0 && score > bestPhaseScore) {
            bestPhaseScore = score;
            bestFirstBeat = candidateFirstBeat;
        }
    }
    
    if (bestFirstBeat == 0) {
        return false;
    }
    
    // === 3. Setze Auto-Maske ===
    m_autoMaskActive = true;
    m_autoMaskFirstBeat = bestFirstBeat;
    m_autoMaskBpm = detectedBpm;
    
    // Lösche alte maskierte Beats und re-evaluiere
    m_maskedRealbeatFrames.clear();
    for (int64_t frame : m_realbeatFrames) {
        if (matchesMask(frame)) {
            m_maskedRealbeatFrames.push_back(frame);
        }
    }
    
    return true;
}

double GridCalculator::calculateMedianInterval() const {
    if (m_realbeatFrames.size() < 2) {
        return 0.0;
    }
    
    std::vector<int64_t> intervals;
    intervals.reserve(m_realbeatFrames.size() - 1);
    
    for (size_t i = 1; i < m_realbeatFrames.size(); ++i) {
        intervals.push_back(m_realbeatFrames[i] - m_realbeatFrames[i-1]);
    }
    
    std::sort(intervals.begin(), intervals.end());
    return static_cast<double>(intervals[intervals.size() / 2]);
}

double GridCalculator::calculateIntervalHistogramPeak(double targetBpm) const {
    if (m_realbeatFrames.size() < 2) {
        return 0.0;
    }
    
    // Berechne alle Intervalle
    std::vector<double> intervals;
    intervals.reserve(m_realbeatFrames.size() - 1);
    
    for (size_t i = 1; i < m_realbeatFrames.size(); ++i) {
        double interval = static_cast<double>(m_realbeatFrames[i] - m_realbeatFrames[i-1]);
        if (interval > 0) {
            intervals.push_back(interval);
        }
    }
    
    if (intervals.empty()) {
        return 0.0;
    }
    
    // Konvertiere BPM-Bereich zu Intervall-Bereich (in Frames)
    double minInterval = 60.0 * m_sampleRate / m_maxBpm;  // Max BPM = Min Interval
    double maxInterval = 60.0 * m_sampleRate / m_minBpm;  // Min BPM = Max Interval
    
    // Wenn TAP-Muster vorgegeben, beschränke Suchbereich auf ±10%
    double targetInterval = 0.0;
    if (targetBpm > 0.0) {
        targetInterval = 60.0 * m_sampleRate / targetBpm;
        minInterval = targetInterval * 0.9;
        maxInterval = targetInterval * 1.1;
    }
    
    // Histogram mit 1ms Bins (44.1 Frames bei 44100 Hz)
    double binSize = m_sampleRate / 1000.0;  // 1ms in Frames
    int numBins = static_cast<int>((maxInterval - minInterval) / binSize) + 1;
    
    if (numBins <= 0 || numBins > 10000) {
        // Fallback: Median
        std::sort(intervals.begin(), intervals.end());
        double medianInterval = intervals[intervals.size() / 2];
        return 60.0 * m_sampleRate / medianInterval;
    }
    
    std::vector<int> histogram(numBins, 0);
    
    // Fülle Histogram
    for (double interval : intervals) {
        // Berücksichtige auch halbe/doppelte Intervalle (Harmonische)
        for (double multiplier : {0.5, 1.0, 2.0}) {
            double adjInterval = interval * multiplier;
            if (adjInterval >= minInterval && adjInterval <= maxInterval) {
                int bin = static_cast<int>((adjInterval - minInterval) / binSize);
                if (bin >= 0 && bin < numBins) {
                    // Gewichtung: 1.0 für Hauptintervall, 0.5 für Harmonische
                    histogram[bin] += (multiplier == 1.0) ? 2 : 1;
                }
            }
        }
    }
    
    // Finde Peak im Histogram
    int maxBin = 0;
    int maxCount = 0;
    for (int i = 0; i < numBins; ++i) {
        if (histogram[i] > maxCount) {
            maxCount = histogram[i];
            maxBin = i;
        }
    }
    
    // Konvertiere Bin zurück zu BPM
    double peakInterval = minInterval + (maxBin + 0.5) * binSize;
    double bpm = 60.0 * m_sampleRate / peakInterval;
    
    return bpm;
}

double GridCalculator::calculatePhase(double bpm, int64_t referenceFrame) const {
    if (m_realbeatFrames.empty() || bpm <= 0.0 || referenceFrame <= 0) {
        return 0.0;
    }
    
    double framesPerBeat = 60.0 * m_sampleRate / bpm;
    
    // Berechne Phase jedes REALBEATs relativ zum Referenz-Frame
    std::vector<double> phases;
    phases.reserve(m_realbeatFrames.size());
    
    for (int64_t beatFrame : m_realbeatFrames) {
        double offset = static_cast<double>(beatFrame - referenceFrame);
        // Normalisiere auf 0.0-1.0
        double phase = std::fmod(offset / framesPerBeat, 1.0);
        if (phase < 0.0) phase += 1.0;
        phases.push_back(phase);
    }
    
    // Berechne zirkulären Mittelwert der Phasen
    // (Phasen sind zirkulär, also 0.0 und 1.0 sind benachbart)
    double sumSin = 0.0;
    double sumCos = 0.0;
    for (double phase : phases) {
        double angle = phase * 2.0 * M_PI;
        sumSin += std::sin(angle);
        sumCos += std::cos(angle);
    }
    
    double meanAngle = std::atan2(sumSin, sumCos);
    double meanPhase = meanAngle / (2.0 * M_PI);
    if (meanPhase < 0.0) meanPhase += 1.0;
    
    return meanPhase;
}

double GridCalculator::calculateConfidence(double bpm) const {
    if (m_realbeatFrames.size() < 3 || bpm <= 0.0) {
        return 0.0;
    }
    
    double framesPerBeat = 60.0 * m_sampleRate / bpm;
    
    // Berechne wie gut die Intervalle zum erkannten BPM passen
    int matchingIntervals = 0;
    int totalIntervals = 0;
    
    for (size_t i = 1; i < m_realbeatFrames.size(); ++i) {
        double interval = static_cast<double>(m_realbeatFrames[i] - m_realbeatFrames[i-1]);
        
        // Prüfe ob Intervall ein Vielfaches der Beat-Periode ist (1x, 2x, 0.5x)
        for (double multiplier : {0.5, 1.0, 2.0}) {
            double expectedInterval = framesPerBeat * multiplier;
            double error = std::abs(interval - expectedInterval) / expectedInterval;
            if (error < 0.1) {  // ±10% Toleranz
                matchingIntervals++;
                break;
            }
        }
        totalIntervals++;
    }
    
    if (totalIntervals == 0) {
        return 0.0;
    }
    
    return static_cast<double>(matchingIntervals) / totalIntervals;
}

// ============================================================================
// TAP-basierte SOLL-GRID Funktionen
// ============================================================================

double GridCalculator::calculateTapBpm() const {
    if (m_tapFrames.size() < 2) {
        return 0.0;
    }
    
    // Berechne Median der TAP-Intervalle
    std::vector<double> intervals;
    intervals.reserve(m_tapFrames.size() - 1);
    
    for (size_t i = 1; i < m_tapFrames.size(); ++i) {
        double interval = static_cast<double>(m_tapFrames[i] - m_tapFrames[i-1]);
        // Nur plausible Intervalle (entsprechend 30-240 BPM)
        double bpmFromInterval = 60.0 * m_sampleRate / interval;
        if (bpmFromInterval >= 30.0 && bpmFromInterval <= 240.0) {
            intervals.push_back(interval);
        }
    }
    
    if (intervals.empty()) {
        return 0.0;
    }
    
    // Median berechnen
    std::sort(intervals.begin(), intervals.end());
    double medianInterval = intervals[intervals.size() / 2];
    
    return 60.0 * m_sampleRate / medianInterval;
}

double GridCalculator::calculateTapPhase(double bpm, int64_t referenceFrame) const {
    if (m_tapFrames.empty() || bpm <= 0.0) {
        return 0.0;
    }
    
    double framesPerBeat = 60.0 * m_sampleRate / bpm;
    
    // Phase basierend auf erstem TAP (= Beat 1)
    if (m_firstTapFrame > 0) {
        double offset = static_cast<double>(m_firstTapFrame - referenceFrame);
        double phase = std::fmod(offset / framesPerBeat, 1.0);
        if (phase < 0.0) phase += 1.0;
        return phase;
    }
    
    return 0.0;
}

double GridCalculator::calculateTapRealbeatOverlap(double tapBpm) const {
    // Berechne wie viele REALBEATs zu den TAP-Zeitpunkten passen
    // Ein REALBEAT "passt" wenn er innerhalb ±10% einer Beat-Periode vom nächsten TAP liegt
    
    if (m_tapFrames.empty() || m_realbeatFrames.empty() || tapBpm <= 0.0) {
        return 0.0;
    }
    
    double framesPerBeat = 60.0 * m_sampleRate / tapBpm;
    double toleranceFrames = framesPerBeat * 0.1;  // ±10%
    
    int matchingRealbeats = 0;
    
    for (int64_t realbeat : m_realbeatFrames) {
        // Finde nächsten TAP
        for (int64_t tap : m_tapFrames) {
            // Berechne Offset in Beat-Perioden
            double offset = static_cast<double>(realbeat - tap);
            double beatOffset = offset / framesPerBeat;
            
            // Runde auf nächsten Beat
            double roundedBeatOffset = std::round(beatOffset);
            double expectedRealbeat = tap + roundedBeatOffset * framesPerBeat;
            double error = std::abs(realbeat - expectedRealbeat);
            
            if (error < toleranceFrames) {
                matchingRealbeats++;
                break;  // Dieser REALBEAT passt, nächsten prüfen
            }
        }
    }
    
    return static_cast<double>(matchingRealbeats) / m_realbeatFrames.size();
}

SollGridResult GridCalculator::calculateSollGrid(int64_t currentFrame, int sampleRate) {
    SollGridResult result;
    
    // Mindestens 2 TAPs für SOLL-GRID (definiert das Muster)
    if (m_tapFrames.size() < 2 || m_tapBpm <= 0.0) {
        return result;
    }
    
    // TAP-BPM muss im gültigen Bereich sein
    if (m_tapBpm < m_minBpm || m_tapBpm > m_maxBpm) {
        return result;
    }
    
    double framesPerBeat = 60.0 * sampleRate / m_tapBpm;
    
    // === SOLL-GRID aus TAP-Muster + MASKED-REALBEATs ===
    // 
    // Das TAP-Muster definiert die Grundfrequenz und den Startpunkt (Beat 1)
    // Die MASKED-REALBEATs (die zur Maske passen) verfeinern die Position
    
    // Nächster erwarteter Beat basierend auf erstem TAP
    double elapsedFromFirstTap = static_cast<double>(currentFrame - m_firstTapFrame);
    double beatsSinceFirstTap = elapsedFromFirstTap / framesPerBeat;
    double nextBeatNumber = std::ceil(beatsSinceFirstTap);
    int64_t nextBeatFrame = m_firstTapFrame + static_cast<int64_t>(nextBeatNumber * framesPerBeat);
    
    // Phase berechnen: Wo sind wir im aktuellen Beat-Zyklus?
    // Phase 0.0 = genau auf dem Beat, Phase 0.5 = halber Beat vor dem nächsten
    double phase = std::fmod(beatsSinceFirstTap, 1.0);
    if (phase < 0.0) phase += 1.0;
    // Invertieren: phase 0.0 = nächster Beat kommt jetzt, phase 0.9 = gerade Beat gewesen
    phase = 1.0 - phase;
    if (phase >= 1.0) phase = 0.0;
    
    // === Konfidenz aus MASKED-REALBEATs ===
    // Je mehr REALBEATs zur Maske passen, desto höher die Konfidenz
    double maskedRatio = 0.0;
    if (!m_realbeatFrames.empty()) {
        maskedRatio = static_cast<double>(m_maskedRealbeatFrames.size()) / m_realbeatFrames.size();
    }
    
    // Basis-Konfidenz aus TAPs + Bonus aus passenden REALBEATs
    double tapConfidence = std::min(1.0, m_tapFrames.size() / 4.0);  // 4 TAPs = 100%
    double realbeatConfidence = maskedRatio;
    result.confidence = 0.5 * tapConfidence + 0.5 * realbeatConfidence;
    
    // === VERFEINERUNG durch MASKED-REALBEATs ===
    // Wenn wir gefilterte REALBEATs haben, nutze den letzten für genauere Phase
    if (!m_maskedRealbeatFrames.empty()) {
        int64_t lastMaskedRealbeat = m_maskedRealbeatFrames.back();
        
        // Wie weit ist der letzte MASKED-REALBEAT vom erwarteten Grid entfernt?
        double realbeatOffset = static_cast<double>(lastMaskedRealbeat - m_firstTapFrame);
        double realbeatBeatNumber = realbeatOffset / framesPerBeat;
        double realbeatError = (realbeatBeatNumber - std::round(realbeatBeatNumber)) * framesPerBeat;
        
        // Verschiebe nextBeatFrame um den Fehler (gewichtet)
        // Dies korrigiert das SOLL-GRID basierend auf tatsächlichen Audio-Beats
        double correctionWeight = std::min(1.0, m_maskedRealbeatFrames.size() / 3.0);
        nextBeatFrame += static_cast<int64_t>(realbeatError * correctionWeight * 0.5);
        
        // Phase neu berechnen mit korrigiertem Grid
        double correctedElapsed = static_cast<double>(currentFrame - (nextBeatFrame - framesPerBeat));
        phase = std::fmod(correctedElapsed / framesPerBeat, 1.0);
        if (phase < 0.0) phase += 1.0;
    }
    
    result.bpm = m_tapBpm;
    result.phase = phase;
    result.nextBeatFrame = nextBeatFrame;
    result.valid = true;
    
    return result;
}

MaskResult GridCalculator::getMaskCorrection(int64_t synthBeatFrame, double synthBpm, int64_t /* currentFrame */) const {
    MaskResult result;
    
    // Mindestens 2 maskierte REALBEATs für sample-genaue Berechnung
    if (m_maskedRealbeatFrames.size() < 2) {
        // Fallback auf TAP-BPM wenn vorhanden
        if (m_tapBpm > 0 && m_maskSlotsFilled >= 2) {
            result.bpm = m_tapBpm;
            result.valid = true;
            result.confidence = 0.5;
            // Keine Phase-Korrektur ohne REALBEATs
            result.bpmCorrection = 0.0;
            result.phaseCorrection = 0.0;
        }
        return result;
    }
    
    // === BPM aus maskierten REALBEATs berechnen (SAMPLE-GENAU) ===
    // Nutze die Intervalle der letzten maskierten REALBEATs
    std::vector<double> intervals;
    for (size_t i = 1; i < m_maskedRealbeatFrames.size(); ++i) {
        double interval = static_cast<double>(m_maskedRealbeatFrames[i] - m_maskedRealbeatFrames[i-1]);
        // Normalisiere auf 1-Beat-Intervall (falls 2-Beat-Sprünge dabei sind)
        if (m_maskIntervalFrames > 0) {
            double beats = std::round(interval / m_maskIntervalFrames);
            if (beats >= 1.0 && beats <= 4.0) {
                intervals.push_back(interval / beats);
            }
        } else if (interval > 0) {
            intervals.push_back(interval);
        }
    }
    
    if (intervals.empty()) {
        return result;
    }
    
    // Median-Intervall für Robustheit
    std::sort(intervals.begin(), intervals.end());
    double medianInterval = intervals[intervals.size() / 2];
    double maskBpm = 60.0 * m_sampleRate / medianInterval;
    
    // Sanity-Check
    if (maskBpm < m_minBpm || maskBpm > m_maxBpm) {
        return result;
    }
    
    result.bpm = maskBpm;
    result.valid = true;
    
    // === BPM-Korrektur (Pitch) ===
    double bpmDiff = maskBpm - synthBpm;
    // Max 0.2 BPM pro Korrektur
    if (bpmDiff > 0.2) bpmDiff = 0.2;
    if (bpmDiff < -0.2) bpmDiff = -0.2;
    result.bpmCorrection = bpmDiff;
    
    // === Phase-Korrektur (Nudge) - SAMPLE-GENAU ===
    // Der letzte maskierte REALBEAT ist unsere Referenz
    int64_t lastRealbeat = m_maskedRealbeatFrames.back();
    
    // Berechne wo der SYNTHBEAT sein sollte relativ zum letzten REALBEAT
    // SYNTHBEAT läuft bei synthBeatFrame, wir wollen wissen: Ist er in Phase?
    double offsetFromRealbeat = static_cast<double>(synthBeatFrame - lastRealbeat);
    double beatsFromRealbeat = offsetFromRealbeat / medianInterval;
    
    // Der nächste SYNTHBEAT nach dem letzten REALBEAT sollte bei:
    // lastRealbeat + ceil(beatsFromRealbeat) * medianInterval sein
    // Aber der SYNTH ist bei synthBeatFrame + elapsed
    
    // Einfacher: Wo ist die Phase des SYNTH relativ zum REALBEAT-Grid?
    double fractionalBeat = beatsFromRealbeat - std::floor(beatsFromRealbeat);
    if (fractionalBeat < 0) fractionalBeat += 1.0;
    
    // Phase-Fehler: 0.0 = genau auf Beat, 0.5 = zwischen Beats, 1.0 = nächster Beat
    double phaseError = fractionalBeat;
    if (phaseError > 0.5) phaseError -= 1.0;  // -0.5 bis +0.5
    
    // Korrektur in Frames
    double correctionFrames = phaseError * medianInterval;
    
    // Max 5ms pro Korrektur für sanftes Driften
    double maxNudge = 0.005 * m_sampleRate;
    if (correctionFrames > maxNudge) correctionFrames = maxNudge;
    if (correctionFrames < -maxNudge) correctionFrames = -maxNudge;
    
    result.phaseCorrection = correctionFrames;
    result.nextBeatFrame = lastRealbeat + static_cast<int64_t>(std::ceil(beatsFromRealbeat) * medianInterval);
    
    // Konfidenz basierend auf Anzahl und Konsistenz der Intervalle
    double intervalVariance = 0.0;
    for (double iv : intervals) {
        double diff = iv - medianInterval;
        intervalVariance += diff * diff;
    }
    intervalVariance /= intervals.size();
    double stdDev = std::sqrt(intervalVariance);
    double relativeStdDev = stdDev / medianInterval;  // 0.0 = perfekt konsistent
    
    result.confidence = std::min(1.0, intervals.size() / 4.0) * std::max(0.0, 1.0 - relativeStdDev * 5.0);
    
    return result;
}

} // namespace BeatAnalyzer
