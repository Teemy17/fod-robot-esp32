#pragma once
#include <cstdint>

// Hardware-independent bench-test state. No automatic motion or repeating runs.
struct MotorTestState {
  static constexpr uint32_t kRunMs = 1000;
  int percent = 20;
  int left = 0, right = 0;  // signed requested duty percentages
  bool fault = false;
  uint32_t startedMs = 0;

  bool moving() const { return left != 0 || right != 0; }
  void stop() { left = right = 0; }
  void tick(uint32_t now, bool estop) {
    if (estop) { fault = true; stop(); }
    if (moving() && uint32_t(now - startedMs) >= kRunMs) stop();
  }
  bool command(char c, uint32_t now, bool estop) {
    tick(now, estop);
    if (c == 'S') { stop(); return true; }
    if (c == 'E') {
      stop();
      if (estop) return false;
      fault = false;
      return true;
    }
    if (c >= '1' && c <= '5') {
      if (moving()) return false;
      percent = (c - '0') * 10;
      return true;
    }
    // A second command cannot extend a running test or reverse it abruptly.
    if (fault || moving()) return false;
    switch (c) {
      case 'F': left = right = percent; break;
      case 'B': left = right = -percent; break;
      case 'L': left = -percent; right = percent; break;
      case 'R': left = percent; right = -percent; break;
      case 'A': left = percent; break;   // left side only
      case 'C': right = percent; break;  // right side only
      default: return false;
    }
    startedMs = now;
    return true;
  }
};
