#pragma once
// VL53L0X obstacle sensing -- PRD FR-10: one front-center (covers the
// lidar's ~15-20cm dead zone) + two front-corner units at 45deg, optional
// downward cliff sensor. Not implemented yet; this header is the seam so
// main.cpp doesn't need to change when it is.

#include <Arduino.h>

// Bring up every sensor on the shared I2C bus, toggling XSHUT pins in turn
// to assign each a distinct address (they all boot at 0x29).
void tofBegin();

// Poll all sensors; returns true if ANY reads below kTofThresholdM. Call
// from a low-priority task at timing::kTofPollHz, not from controlTask --
// I2C transactions are too slow to share the 50 Hz control tick budget.
bool tofObstacleDetected();
