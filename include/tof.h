#pragma once
// PRD v2: lateral + low-forward VL53L0X. Interface only; not implemented.

#include <Arduino.h>

// Bring up every sensor on the shared I2C bus, toggling XSHUT pins in turn
// to assign each a distinct address (they all boot at 0x29).
void tofBegin();

// Poll all sensors; returns true if ANY reads below kTofThresholdM. Call
// from a low-priority task at timing::kTofPollHz, not from controlTask --
// I2C transactions are too slow to share the 50 Hz control tick budget.
bool tofObstacleDetected();
