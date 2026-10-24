#include <cassert>
#include <cmath>
#include <cstdio>
#include "drive_control.h"
#include "pid.h"
#include "quadrature.h"

// Same numbers as include/robot_v1.h (kept literal so this test needs no Arduino headers).
constexpr float kFullScale = 1023.0f;
constexpr float kNoLoadMps = 175.0f / 60.0f * 2.0f * drive::kPi * 0.0325f;
constexpr float kFF = 0.75f * kFullScale / kNoLoadMps;  // robot::kFeedforward
constexpr float dt = 0.02f;

// First-order wheel as the firmware sees it: one control tick of actuation delay, then a
// first-order lag, then the same low-pass (alpha 0.3) the firmware applies to the speed.
struct Plant {
  float speed = 0.0f;     // true wheel speed
  float measured = 0.0f;  // what the PID sees
  float gain = kNoLoadMps / kFullScale;  // m/s per count
  float tau = 0.15f;
  float pending = 0.0f;
  float step(float counts) {
    const float applied = pending;
    pending = counts;
    speed += dt * (gain * applied - speed) / tau;
    measured += 0.3f * (speed - measured);
    return speed;
  }
};

Pid makePid(float outMax) {
  Pid p;
  p.kp = 700.0f; p.ki = 3000.0f; p.kd = 0.0f; p.kff = kFF; p.outMax = outMax;
  return p;
}

static bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

int main() {
  // --- quadrature decode -------------------------------------------------------------
  {
    const uint8_t fwd[] = {0, 2, 3, 1};  // A leads B: 00 -> 10 -> 11 -> 01
    int32_t ticks = 0;
    uint8_t prev = 1;  // the last state of the sequence, so every cycle is a full +4
    for (int cycle = 0; cycle < 100; ++cycle)
      for (uint8_t s : fwd) { ticks += quadDelta(prev, s); prev = s; }
    assert(ticks == 400);  // x4: four counts per cycle, no illegal transitions
    const uint8_t rev[] = {1, 3, 2, 0};  // the same sequence backwards
    prev = 0;                            // last state of rev
    for (int cycle = 0; cycle < 100; ++cycle)
      for (uint8_t s : rev) { ticks += quadDelta(prev, s); prev = s; }
    assert(ticks == 0);
    assert(quadDelta(0, 0) == 0 && quadDelta(3, 3) == 0);
    assert(quadIllegal(0, 3) && quadIllegal(3, 0) && quadIllegal(1, 2) && quadIllegal(2, 1));
    assert(!quadIllegal(0, 1) && !quadIllegal(0, 2) && !quadIllegal(2, 2));
    assert(quadDelta(0, 3) == 0 && quadDelta(1, 2) == 0);  // skipped state: counted, not added
  }

  // --- unicycle -> wheels ---------------------------------------------------------------
  {
    float l, r;
    drive::unicycleToWheels(0.2f, 0.0f, 0.2f, l, r);
    assert(near(l, 0.2f, 1e-6f) && near(r, 0.2f, 1e-6f));
    drive::unicycleToWheels(0.0f, 1.0f, 0.2f, l, r);  // CCW: right wheel faster
    assert(near(l, -0.1f, 1e-6f) && near(r, 0.1f, 1e-6f));
    drive::unicycleToWheels(0.2f, 1.0f, 0.2f, l, r);
    assert(near(l, 0.1f, 1e-6f) && near(r, 0.3f, 1e-6f));
  }

  // --- speed filter -----------------------------------------------------------------------
  {
    drive::SpeedFilter f;
    float y = 0;
    for (int i = 0; i < 40; ++i) y = f.update(1.0f, 0.3f);
    assert(near(y, 1.0f, 1e-4f));
    f.reset();
    assert(f.value == 0.0f);
  }

  // --- PID: step response ------------------------------------------------------------------
  {
    const float outMax = 0.5f * kFullScale;
    Pid pid = makePid(outMax);
    Plant plant;
    float maxSpeed = 0.0f, out = 0.0f, settled = -1.0f;
    const float target = 0.20f;
    for (int i = 0; i < 100; ++i) {  // 2 s
      out = pid.update(target, plant.measured, dt);
      assert(std::fabs(out) <= outMax + 1e-3f);
      const float s = plant.step(out);
      if (s > maxSpeed) maxSpeed = s;
      if (settled < 0 && near(s, target, 0.02f * target)) settled = i * dt;
    }
    std::printf("step 0.20 m/s: settled(2%%) at %.2f s, peak %.3f m/s\n", settled, maxSpeed);
    assert(settled >= 0.0f && settled < 0.6f);
    assert(maxSpeed < target * 1.10f);               // < 10% overshoot
    assert(near(plant.speed, target, 0.002f));        // steady state
  }

  // --- PID: reverse and a change of setpoint -----------------------------------------------
  {
    const float outMax = 0.5f * kFullScale;
    Pid pid = makePid(outMax);
    Plant plant;
    for (int i = 0; i < 100; ++i) plant.step(pid.update(0.20f, plant.measured, dt));
    for (int i = 0; i < 150; ++i) plant.step(pid.update(-0.15f, plant.measured, dt));
    assert(near(plant.speed, -0.15f, 0.005f));
  }

  // --- PID: no wind-up when the setpoint is unreachable ----------------------------------------
  {
    const float outMax = 0.5f * kFullScale;  // max reachable speed is about 0.30 m/s
    Pid pid = makePid(outMax);
    Plant plant;
    for (int i = 0; i < 150; ++i) plant.step(pid.update(0.60f, plant.measured, dt));  // 3 s saturated
    assert(plant.speed < 0.31f);
    assert(std::fabs(pid.integral * pid.ki) <= outMax + 1e-3f);
    float settled = -1.0f;
    for (int i = 0; i < 50; ++i) {  // then ask for something reachable
      const float s = plant.step(pid.update(0.20f, plant.measured, dt));
      if (settled < 0 && near(s, 0.20f, 0.01f)) settled = i * dt;
    }
    std::printf("recovery after saturation: within 5%% at %.2f s\n", settled);
    assert(settled >= 0.0f && settled < 0.5f);
  }

  // --- PID: ki = 0 must not produce NaN or inf -------------------------------------------------
  {
    Pid pid = makePid(511.0f);
    pid.ki = 0.0f;
    Plant plant;
    for (int i = 0; i < 100; ++i) {
      const float out = pid.update(0.2f, plant.measured, dt);
      assert(std::isfinite(out));
      plant.step(out);
    }
    assert(pid.integral == 0.0f);
  }

  // --- PID: derivative on the measurement does not kick on a setpoint change --------------------
  {
    Pid pid = makePid(511.0f);
    pid.kp = 0.0f; pid.ki = 0.0f; pid.kff = 0.0f; pid.kd = 100.0f;
    pid.update(0.0f, 0.1f, dt);
    const float out = pid.update(0.3f, 0.1f, dt);  // setpoint jumped, measurement did not
    assert(out == 0.0f);
  }

  // --- stall detector ----------------------------------------------------------------------------
  {
    drive::StallDetector d;
    int ticksToTrip = 0;
    for (int i = 1; i <= 100; ++i) {
      if (d.update(300.0f, 0.0f, 20, 250.0f, 0.01f, 600)) { ticksToTrip = i; break; }
    }
    assert(ticksToTrip == 30);  // 30 x 20 ms = 600 ms
    d.reset();
    for (int i = 0; i < 29; ++i) assert(!d.update(300.0f, 0.0f, 20, 250.0f, 0.01f, 600));
    assert(!d.update(300.0f, 0.2f, 20, 250.0f, 0.01f, 600));   // it moved: counter resets
    for (int i = 0; i < 29; ++i) assert(!d.update(300.0f, 0.0f, 20, 250.0f, 0.01f, 600));
    d.reset();
    for (int i = 0; i < 200; ++i) assert(!d.update(100.0f, 0.0f, 20, 250.0f, 0.01f, 600));  // low duty is not a stall
  }

  std::puts("control_test: ok");
}
