// A tap fixes BTrack's tempo; after TAP_LOCK_BARS bars the lock lets go and
// detection follows the music again. A new tap starts the count over.
// Decided 2026-10-08 (a3-doc open question 17): before, one tap fixed the
// tempo until the service restarted.
#include <cassert>
#include <iostream>

#include "../include/analysis/tap_tempo_lock.h"

using namespace BeatAnalyzer;

static int beatsUntilRelease(TapTempoLock& lock, int limit)
{
    for (int beat = 1; beat <= limit; ++beat)
        if (lock.beat())
            return beat;
    return -1;
}

static void test_nothing_is_held_before_a_tap()
{
    TapTempoLock lock(16);
    assert(!lock.locked());
    assert(!lock.beat());
    std::cout << "  ✓ nothing is held before a tap" << std::endl;
}

static void test_a_tap_holds_for_sixteen_bars_then_lets_go_once()
{
    TapTempoLock lock(16);
    lock.lock();
    assert(lock.locked());
    assert(beatsUntilRelease(lock, 1000) == 16 * 4);
    assert(!lock.locked());
    assert(!lock.beat());
    std::cout << "  ✓ a tap holds for 16 bars, then lets go once" << std::endl;
}

static void test_a_new_tap_starts_the_count_over()
{
    TapTempoLock lock(16);
    lock.lock();
    for (int beat = 0; beat < 60; ++beat)
        assert(!lock.beat());
    lock.lock();
    assert(beatsUntilRelease(lock, 1000) == 16 * 4);
    std::cout << "  ✓ a new tap starts the count over" << std::endl;
}

static void test_the_bar_count_is_a_setting()
{
    TapTempoLock lock(2);
    lock.lock();
    assert(beatsUntilRelease(lock, 1000) == 2 * 4);
    std::cout << "  ✓ the number of bars is a setting" << std::endl;
}

static void test_zero_bars_keeps_the_lock_until_restart()
{
    TapTempoLock lock(0);
    lock.lock();
    assert(beatsUntilRelease(lock, 10000) == -1);
    assert(lock.locked());
    std::cout << "  ✓ 0 bars keeps the lock until restart" << std::endl;
}

int main()
{
    std::cout << "TapTempoLock" << std::endl;
    test_nothing_is_held_before_a_tap();
    test_a_tap_holds_for_sixteen_bars_then_lets_go_once();
    test_a_new_tap_starts_the_count_over();
    test_the_bar_count_is_a_setting();
    test_zero_bars_keeps_the_lock_until_restart();
    return 0;
}
