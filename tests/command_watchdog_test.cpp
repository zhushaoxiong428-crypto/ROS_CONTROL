#include "command_watchdog.h"

#include <cassert>
#include <cstdint>

int main() {
    CommandWatchdog watchdog(500);

    // Nothing has been commanded yet, so there is nothing to keep alive.
    assert(!watchdog.IsArmed());
    assert(!watchdog.IsFresh(0));

    // A command stays valid up to and including the timeout, then expires.
    watchdog.Feed(1000);
    assert(watchdog.IsArmed());
    assert(watchdog.IsFresh(1000));
    assert(watchdog.IsFresh(1500));
    assert(!watchdog.IsFresh(1501));

    // A new command restarts the window.
    watchdog.Feed(1400);
    assert(watchdog.IsFresh(1900));
    assert(!watchdog.IsFresh(1901));

    // Disarming after a timeout stop keeps it expired until the next command.
    watchdog.Disarm();
    assert(!watchdog.IsArmed());
    assert(!watchdog.IsFresh(1400));
    watchdog.Feed(2000);
    assert(watchdog.IsFresh(2100));

    // The millisecond tick wraps after ~49.7 days; age is still measured correctly.
    const uint32_t near_wrap = UINT32_MAX - 100;
    watchdog.Feed(near_wrap);
    assert(watchdog.IsFresh(near_wrap + 300));  // wraps past zero
    assert(watchdog.IsFresh(399));               // exactly 500 ms later
    assert(!watchdog.IsFresh(400));

    // Only modes that hold a speed need the watchdog.
    assert(IsStreamingMotionMode(0));
    assert(!IsStreamingMotionMode(1));
    assert(!IsStreamingMotionMode(2));
    assert(IsStreamingMotionMode(3));
    assert(IsStreamingMotionMode(6));
    assert(IsStreamingMotionMode(7));
}
