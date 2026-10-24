#pragma once
// Pure quadrature-decode helpers. No Arduino dependency, so they are host-testable
// (tests/host/control_test.cpp).
//
// State is (A << 1) | B. Table index is (previous << 2) | current.
// Convention: +1 when A leads B (00 -> 10 -> 11 -> 01 -> 00). Which physical rotation is
// "forward" depends on wiring, so that is fixed by robot::kLeftEncSign / kRightEncSign,
// never here.
//
// A transition where A and B both change cannot happen in a clean signal. It decodes as 0
// and quadIllegal() reports it so the caller can count it as noise instead of silently
// losing the tick.

#include <stdint.h>

constexpr int8_t kQuadTable[16] = { 0, -1,  1,  0,
                                    1,  0,  0, -1,
                                   -1,  0,  0,  1,
                                    0,  1, -1,  0 };

inline int8_t quadDelta(uint8_t prev, uint8_t cur) {
  return kQuadTable[((prev & 3) << 2) | (cur & 3)];
}

inline bool quadIllegal(uint8_t prev, uint8_t cur) {
  return (((prev ^ cur) & 3) == 3);
}
