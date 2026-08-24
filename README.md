# fod-robot-esp32

Motor + sensor firmware for the FOD robot's ESP32, PlatformIO + Arduino
framework. This is the third repo in the project, alongside:

- **`fod-robot-pathing`** — the Pi-side nav stack. Owns `docs/protocol.md`,
  the contract this firmware implements. Its simulator (`fodnav-sim`) fakes
  this whole board in software (`sim/firmware.py`) so nav can be built and
  tested with no ESP32 at all — that fake is a second, independent
  implementation of the same protocol, and a good thing to diff your
  firmware's behaviour against if something doesn't match.
- **`fod-robot-cv-poc`** — the camera/detector pipeline. Runs on the Pi, has
  no contact with this firmware; it publishes detections over MQTT for
  `fodnav` to consume, not to the ESP32.

**Read `docs/protocol.md` (copy it from `fod-robot-pathing` into `docs/`
here, verbatim — its own header asks for this) before changing anything in
`src/protocol.cpp` or `src/main.cpp`.** It is a two-repo contract: the Pi's
`tests/test_esp32.py` encodes exactly what it expects a real board to do,
and that test suite is the closest thing to a spec this firmware has.

## Layout

```
platformio.ini
include/
  config.h       pins + measured constants (this repo's config/robot.yaml)
  protocol.h     the wire codec — mirrors fodnav/link/esp32.py
  encoders.h     quadrature ISR tick counting
  motor.h        TB6612FNG channel wrapper
  pid.h          per-wheel velocity PID
  tof.h          VL53L0X obstacle sensing (stub, not wired yet)
src/
  main.cpp       setup() + two FreeRTOS tasks (comm, control)
  protocol.cpp
  encoders.cpp
  motor.cpp
```

## What's implemented

- The full `docs/protocol.md` v1 line codec, both directions: `V/S/E/D/M/?/H`
  in, `T/I/L` out. Framing rules (120-byte cap, 3 decimals, no exponents,
  malformed-line-dropped-whole, unrecognised-tag-ignored) match the Pi's
  parser field for field.
- The watchdog exactly as specified: arms on the first command, not at
  power-on; 300 ms of silence zeroes velocity and sets flag bit 1; recovery
  needs an explicit `E`.
- Two FreeRTOS tasks (`commTask` on core 0, `controlTask` on core 1 at a
  fixed 50 Hz) plus encoder ISRs that pre-empt both — see CLAUDE.md section 5
  in `fod-robot-pathing` for why a bare `loop()` doesn't work here.
- Per-wheel velocity PID (FR-5) — open-loop PWM was ruled out because magnet
  drag rises as debris accumulates mid-trial, and that's the exact axis the
  evaluation is plotted against.
- Status flags, `T` telemetry every tick unconditionally (symmetric to the
  Pi's own "never skip a `V`" obligation).

## What's stubbed and needs you

Everything hardware-specific and everything the PRD marks 🔶 OPEN / 📏
MEASURE is deliberately left as a `TODO` rather than guessed at — same
discipline as `config/robot.yaml` shipping full of `null` on the Pi side:

- **`include/config.h`** — every pin and every measured constant is a
  placeholder. Fill `robot::kTicksPerRev` / `kWheelRadiusM` / `kTrackWidthM`
  from `docs/HARDWARE.md` §2.1–2.3 in the Pi repo, and **update
  `config/robot.yaml` there the same day** — the `I` handshake asserts the
  two sides agree to 1e-6 m and refuses to run otherwise
  (`Esp32Link.verify()`).
- **Drum motor / RPM** — currently open-loop at a fixed placeholder duty.
  PRD O-4 (drum diameter, magnet arc angle) and M-2 (RPM vs release
  reliability) are still open; if you add an encoder to the drum motor
  (`pins::kDrumEncA`, `HARDWARE.md` recommends the encoded JGB37-520
  variant), close the loop the same way the drive wheels are.
- **`tof.h`/`tof.cpp`** — VL53L0X bring-up (XSHUT address assignment,
  polling, threshold) isn't written yet. `gObstacle` in `main.cpp` is a
  fixed `false` until you wire it in; do it from a low-priority task, not
  from `controlTask`, since I2C reads are too slow for the 50 Hz budget.
- **PID gains** (`config.h` `pid::` namespace) are unturned placeholders.
- **`kFlagEncoderFault`, `kFlagBatteryLow`, `gDriverFault`** — not populated
  yet; wire up when the corresponding sensing exists.

## Build

```
pio run                 # build
pio run -t upload       # flash
pio device monitor       # raw wire traffic at 115200 — readable by eye,
                          # which is the whole reason protocol.md chose ASCII
```

## Testing against the Pi side without hardware

`fod-robot-pathing`'s `fodnav-teleop --sim` and `fodnav-sim` exercise the
*Pi's* side of this protocol against a fake firmware, not this one. To test
*this* firmware for real, the fastest path is:

1. Flash this to the board, connect over USB.
2. On the Pi (or a laptop), `fodnav-teleop --port /dev/ttyUSB0` (or the
   `/dev/serial/by-id/...` path — see `docs/HARDWARE.md` §6, never
   `/dev/ttyACM0`) and drive it by hand.
3. Do the watchdog kill-test for real: drive forward, `kill -9` the Pi
   process, confirm the wheels stop within 300 ms. `docs/protocol.md`
   section 6 calls this out explicitly — do it before the exam, not during.
