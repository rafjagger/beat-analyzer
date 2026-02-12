#pragma once

/**
 * GridCalculator - MASKE für Beat-Synchronisation
 * 
 * VEREINFACHTE ARCHITEKTUR (DJ-Style):
 * 
 * AUDIO → REALBEAT → MASKE → SYNTHBEAT
 *                ↑
 *              TAP
 * 
 * Die MASKE:
 * - Ohne TAP: Aligniert sich automatisch an stärksten REALBEATs
 * - Mit TAP: Spalten auf TAPs setzen, neues Intervall = TAP-Intervall
 * - Filtert REALBEATs: Nur die zur Maske passen werden akzeptiert
 * - Korrigiert sich kontinuierlich anhand gefilterter REALBEATs
 * - Liefert direkt BPM + Phase für SYNTHBEAT
 * 
 * SYNTHBEAT:
 * - Bekommt BPM + Phase von der Maske
 * - Driftet sanft zum Sollwert (DJ-style: Pitch + Nudge)
 */

#include <deque>
#include <vector>
#include <cstdint>

namespace BeatAnalyzer {

/**
 * Ergebnis der Maske - direkt für SYNTHBEAT nutzbar
 */
struct MaskResult {
    double bpm = 0.0;              // Aktuelles Tempo der Maske
    double phaseCorrection = 0.0;  // Phase-Korrektur in Frames (positiv = voraus, negativ = hinten)
    double bpmCorrection = 0.0;    // BPM-Korrektur (positiv = schneller, negativ = langsamer)
    double confidence = 0.0;       // Konfidenz (0.0-1.0)
    bool valid = false;            // Ist die Maske gültig?
    int64_t nextBeatFrame = 0;     // Nächster erwarteter Beat-Frame
};

/**
 * Legacy-Structs für Kompatibilität
 */
struct GridResult {
    double gridBpm = 0.0;
    double gridPhase = 0.0;
    double confidence = 0.0;
    bool valid = false;
    double sollBpm = 0.0;
    double sollPhase = 0.0;
    bool hasSollGrid = false;
};

struct SollGridResult {
    double bpm = 0.0;
    double phase = 0.0;
    double confidence = 0.0;
    int64_t nextBeatFrame = 0;
    bool valid = false;
};

/**
 * GridCalculator Klasse
 */
class GridCalculator {
public:
    /**
     * Konstruktor
     * @param sampleRate Sample-Rate in Hz
     * @param windowSeconds Fenster für REALBEAT-Sammlung in Sekunden
     * @param minBpm Minimales erwartetes Tempo
     * @param maxBpm Maximales erwartetes Tempo
     */
    GridCalculator(int sampleRate, double windowSeconds, double minBpm, double maxBpm);
    
    /**
     * Fügt einen REALBEAT-Timestamp hinzu
     * Entfernt automatisch alte Timestamps außerhalb des Fensters
     * @param frame Frame-Position des erkannten Beats
     * @param currentFrame Aktueller Frame für Fenster-Berechnung
     * @param strength Stärke des Beats (0.0-1.0, für Auto-Maske)
     * @return true wenn REALBEAT zur TAP-Maske passt (oder keine Maske aktiv)
     */
    bool addRealbeat(int64_t frame, int64_t currentFrame, float strength = 1.0f);
    
    /**
     * Prüft ob ein REALBEAT zur aktiven Maske passt (TAP oder Auto)
     * @param realbeatFrame Frame des REALBEATs
     * @return true wenn zur Maske passt oder keine Maske aktiv
     */
    bool matchesMask(int64_t realbeatFrame) const;
    
    /**
     * Prüft ob ein REALBEAT zur TAP-Maske passt
     * @param realbeatFrame Frame des REALBEATs
     * @return true wenn innerhalb ±15% eines Beat-Intervalls vom nächsten erwarteten Beat
     */
    bool matchesTapMask(int64_t realbeatFrame) const;
    
    /**
     * Fügt einen TAP-Timestamp hinzu
     * @param frame Frame-Position des TAPs
     * @param currentFrame Aktueller Frame für Fenster-Berechnung
     * @param isFirstTap True wenn dies der erste TAP nach Reset ist
     */
    void addTap(int64_t frame, int64_t currentFrame, bool isFirstTap = false);
    
    /**
     * Berechnet das Grid aus den gesammelten REALBEATs und TAPs
     * @param tapPatternBpm Optionales TAP-Muster als Suchvorgabe (0.0 = kein Muster)
     * @param referenceFrame Referenz-Frame für Phase-Berechnung (z.B. letzter SYNTHBEAT)
     * @return GridResult mit gridBpm, gridPhase, confidence, und SOLL-GRID
     */
    GridResult calculateGrid(double tapPatternBpm = 0.0, int64_t referenceFrame = 0);
    
    /**
     * Berechnet das SOLL-GRID aus TAP-Muster und gefilterten REALBEATs
     * @param currentFrame Aktueller Frame für Phasen-Berechnung
     * @param sampleRate Sample-Rate für Frame/Zeit Konversion
     * @return SollGridResult mit bpm, phase (relativ zu currentFrame), confidence
     */
    SollGridResult calculateSollGrid(int64_t currentFrame, int sampleRate);
    
    /**
     * Berechnet automatische 4/4-Maske basierend auf lautesten REALBEATs
     * Wird verwendet wenn kein TAP vorhanden ist
     * @param realbeatStrengths Stärken der REALBEATs (parallel zu m_realbeatFrames)
     * @param currentFrame Aktueller Frame
     * @param sampleRate Sample-Rate
     * @return true wenn Maske erfolgreich erstellt wurde
     */
    bool calculateAutoMask(int64_t currentFrame, int sampleRate);
    
    /**
     * Gibt die gefilterten (maskierten) REALBEATs zurück
     */
    size_t getMaskedRealbeatCount() const { return m_maskedRealbeatFrames.size(); }
    
    /**
     * Prüft ob TAPs vorhanden sind
     */
    bool hasTaps() const { return !m_tapFrames.empty(); }
    
    /**
     * Prüft ob Auto-Maske aktiv ist
     */
    bool hasAutoMask() const { return m_autoMaskActive; }
    
    /**
     * Setzt den GridCalculator zurück
     */
    void reset();
    
    /**
     * Gibt die Anzahl der gesammelten REALBEATs zurück
     */
    size_t getRealbeatCount() const { return m_realbeatFrames.size(); }
    
    /**
     * Gibt die Anzahl der gesammelten TAPs zurück
     */
    size_t getTapCount() const { return m_tapFrames.size(); }
    
    /**
     * Gibt den Frame des ersten TAPs zurück (für Beat-1-Sync)
     * @return Frame des ersten TAPs oder 0 wenn kein TAP vorhanden
     */
    int64_t getFirstTapFrame() const { return m_firstTapFrame; }
    
    /**
     * Setzt die Anzahl der Masken-Slots (Standard: 8)
     * @param count Anzahl der Slots (1-32)
     */
    void setMaskSlotCount(int count) { m_maskSlotCount = std::max(1, std::min(32, count)); }
    
    /**
     * Gibt das aktuelle BPM der Maske zurück
     * @return BPM der Maske (TAP oder Auto) oder 0 wenn keine Maske aktiv
     */
    double getMaskBpm() const { 
        if (m_tapBpm > 0) return m_tapBpm;
        if (m_autoMaskBpm > 0) return m_autoMaskBpm;
        return 0.0;
    }
    
    /**
     * Berechnet Korrekturwerte für SYNTHBEAT (DJ-style)
     * @param synthBeatFrame Aktueller SYNTHBEAT-Frame
     * @param synthBpm Aktuelles SYNTHBEAT-BPM
     * @param currentFrame Aktueller Audio-Frame
     * @return MaskResult mit Korrekturwerten
     */
    MaskResult getMaskCorrection(int64_t synthBeatFrame, double synthBpm, int64_t currentFrame) const;
    
private:
    int m_sampleRate;
    double m_windowSeconds;
    double m_minBpm;
    double m_maxBpm;
    int64_t m_windowFrames;
    
    // REALBEAT-Timestamps (alle) mit Stärke
    std::deque<int64_t> m_realbeatFrames;
    std::deque<float> m_realbeatStrengths;  // Stärke jedes REALBEATs
    
    // MASKED-REALBEAT-Timestamps (nur die zur Maske passen)
    std::deque<int64_t> m_maskedRealbeatFrames;
    
    // TAP-Timestamps (definieren die Maske)
    std::deque<int64_t> m_tapFrames;
    int64_t m_firstTapFrame = 0;  // Erster TAP für Beat-1-Sync
    double m_tapBpm = 0.0;        // Berechnetes BPM aus TAP-Intervallen
    
    // Adaptive Maske (für N Beats, Standard: 8 = 2 Takte)
    // Spalt 1 = Beat 1 (erster TAP), weitere werden progressiv gefüllt/vorhergesagt
    int64_t m_maskSlots[32] = {0};            // Frame-Positionen der Slots (max 32)
    int m_maskSlotCount = 8;                  // Anzahl aktiver Slots (aus Config)
    int m_maskSlotsFilled = 0;                // Wie viele Slots gefüllt sind
    double m_maskIntervalFrames = 0.0;        // Beat-Intervall in Frames
    double m_maskTolerancePercent = 0.12;     // Spaltbreite (startet bei ±12%, wird enger)
    int m_consecutiveMisses = 0;              // Konsekutive REALBEATs die nicht zur Maske passen
    
    // Auto-Maske (wenn kein TAP)
    bool m_autoMaskActive = false;
    int64_t m_autoMaskFirstBeat = 0;  // Erster Beat der Auto-Maske
    double m_autoMaskBpm = 0.0;       // BPM der Auto-Maske
    
    // Hilfsfunktionen
    double calculateMedianInterval() const;
    double calculateIntervalHistogramPeak(double targetBpm = 0.0) const;
    double calculatePhase(double bpm, int64_t referenceFrame) const;
    double calculateConfidence(double bpm) const;
    
    // TAP-basierte SOLL-GRID Berechnung
    double calculateTapBpm() const;
    double calculateTapPhase(double bpm, int64_t referenceFrame) const;
    double calculateTapRealbeatOverlap(double tapBpm) const;
    
    // Masken-Selbstkorrektur
    void adjustMaskFromRealbeats();
    void recalculateMaskFromRealbeats();
};

} // namespace BeatAnalyzer
