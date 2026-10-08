#pragma once

namespace BeatAnalyzer {

// How long a tapped tempo holds BTrack. A tap fixes the tempo; after
// barsToHold bars of the beat clock the caller releases it and detection
// follows the music again. A new tap starts the count over. 0 bars keeps the
// lock until the service restarts, as it was before 2026-10-08.
class TapTempoLock {
public:
    explicit TapTempoLock(int barsToHold, int beatsPerBar = 4)
        : m_beatsToHold(barsToHold > 0 ? barsToHold * beatsPerBar : 0) {}

    void lock()
    {
        m_locked = true;
        m_beatsHeld = 0;
    }

    // One beat of the clock. True exactly once: on the beat the lock ends.
    bool beat()
    {
        if (!m_locked || m_beatsToHold == 0)
            return false;
        if (++m_beatsHeld < m_beatsToHold)
            return false;
        m_locked = false;
        return true;
    }

    bool locked() const { return m_locked; }

private:
    int m_beatsToHold;
    int m_beatsHeld = 0;
    bool m_locked = false;
};

} // namespace BeatAnalyzer
