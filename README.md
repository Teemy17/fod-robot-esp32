# FOD robot — yellow TT prototype

ESP32 motor firmware using PlatformIO, Arduino and two FreeRTOS tasks.
This build follows the user's prototype scope: **yellow TT motors without
encoders, no magnetic drum**. PRD v2 describes the eventual robot; its closed-loop
speed and pickup requirements do not apply to this temporary bench build.

Drive is open-loop PWM. There is no measured speed, wheel odometry, or velocity
PID. Encoder sources are retained for future work but excluded from the build.
No drum pins are configured and no collector command is implemented.

## Wiring and configuration

The existing TB6612FNG driver is assumed. Confirm your actual driver and motor
ratings before wiring. Supply VM at the voltage specified for your TT motors;
do not connect the PRD's 3S battery directly to them. A PWM limit does not replace
a suitable motor supply. Check total stall current if pairing motors per side.
Use a common ground between ESP32, driver and motor supply.

| Signal | ESP32 GPIO |
|---|---:|
| Left PWMA / AIN1 / AIN2 | 25 / 26 / 27 |
| Right PWMB / BIN1 / BIN2 | 14 / 12 / 13 |
| STBY | 23 |
| E-stop sense, active low | 19 |

Wire STBY to GPIO 23, not permanently high; add an external pulldown so the
driver stays disabled during boot. GPIO 12 is a boot-strapping pin: external
wiring must not pull it high during reset. E-stop sense uses INPUT_PULLUP and
is grounded when pressed; the physical switch must also cut motor power.
An unconnected sense input reads released, so it does not detect broken wiring.

Set the pins, `prototype::kMaxDuty` (default 0.5), and motor direction signs in
`include/config.h`. Pin assignments are starting wiring choices, not verified
measurements. No wheel diameter, RPM or encoder constants are fabricated.

## Prototype serial protocol

115200 baud, newline-terminated ASCII. Each line is at most 120 bytes including
newline. Numeric fields are finite decimals with at most three decimal places.
Malformed/overlong lines are dropped whole. Commands:

| Command | Effect |
|---|---|
| `H` | `I 2 tt-prototype-0.2.0 0.000 0.000 0.000` |
| `E` | Enable at zero duty; clear faults only if e-stop is released |
| `P <left> <right>` | Signed normalized PWM; each side clamped to ±0.5 by default |
| `S` | Zero both PWM outputs |
| `D` | Zero outputs and disable driver via STBY |
| `?` | Request current prototype status |

Positive duty means forward as wired; opposite signs turn on the spot.
`P` while disabled or faulted is discarded. Send `E`, then fresh `P` commands.
`V` is rejected with a warning because there is no velocity feedback. Legacy
`M` and PRD `DRUM` commands are unrecognized and ignored, without feeding the
watchdog.

Status is sent at 50 Hz and on `?`:

```
PT <ms> <applied_left_pwm> <applied_right_pwm> <enabled> <watchdog> <estop> <clamped>
```

PWM values are signed hardware duties (10-bit, full scale 1023), including
configured direction signs. Boolean fields are 0/1. These are commanded
outputs, not evidence that a wheel moved. No `T`, `VEL` or `ODO` readings are
emitted. Handshake zero fields mean geometry/encoders unavailable.

The watchdog arms on the first valid `P/S/E/D/?`. A 300 ms command timeout
stops outputs and disables the driver on the next 20 ms control tick (up to
about 320 ms plus scheduling delay). E-stop and watchdog faults are latched:
release the switch, send `E`, then send fresh `P` commands. Query commands feed
the watchdog, so a controller must not keep querying after its motion loop fails.
`S` takes effect on the next control tick. `D`/faults use standby (coast), not a
promise of a particular mechanical stopping distance.

**Pi compatibility:** this is a prototype dialect, version 2, not an update to
the shared navigation protocol. Existing `fod-robot-pathing` v1 must reject its
handshake. Its navigation and teleop require real encoder odometry and cannot
be used unchanged with this build. Use the bench script below instead. Restore
encoder hardware/control and coordinate the shared protocol before autonomous
navigation or PRD speed experiments.

## Sensors

ToF polling was already unimplemented and remains so; `include/tof.h` is only
an interface placeholder. PRD v2 calls for two VL53L0X units (lateral and low
forward) plus a bump switch. This build does not yet read them or perform
obstacle stops. E-stop sense is the only implemented sensor input. Battery
and driver current sensing are also absent.

## Build and bench test

```sh
pio run
pio run -t upload
python3 tools/tt_drive.py --port /dev/cu.YOUR_ESP32 --left 0.2 --right 0.2 --seconds 1
```

The script requires pyserial, verifies the prototype handshake, enables at zero,
sends PWM at 50 Hz, then stops/disables in a `finally` block. Start with wheels
raised. If duty 0.2 cannot overcome friction, zero output first and increase
carefully within the configured limit; duty is not a calibrated speed.

On hardware verify each wheel's direction, `S`, `D`, e-stop latching, and the
watchdog by terminating the sender abruptly. Confirm releasing e-stop or
reconnecting alone does not restart the wheels. Compilation does not validate
motor wiring, current limits or physical stopping behavior.

Host protocol regression checks (no board required):

```sh
c++ -std=c++17 -Itests/host -Iinclude src/protocol.cpp tests/host/protocol_test.cpp -o /tmp/fod-tt-protocol-test
/tmp/fod-tt-protocol-test
```

## Standalone motor test — no Raspberry Pi

Select the separate `motor_test` firmware. Connect your computer directly to
ESP32 USB for power and commands. The four-AA pack powers both drivers' VM;
ESP32 3V3 powers their VCC. Join battery negative, driver GND and ESP32 GND.
Disconnect the Pi and do not add another ESP32 power feed.

With two TB6612 boards, share the control inputs listed in `include/config.h`:
GPIO 25/26/27 to PWMA/AIN1/AIN2 on both boards, GPIO 14/12/13 to
PWMB/BIN1/BIN2 on both boards, and GPIO 23 to both STBY pins. Board 1 drives
front left/right on A/B; board 2 drives rear left/right on A/B. Each motor
gets its own channel; do not join motor outputs. This tests wheel sides,
not each of the four motors independently.

```sh
pio run -e motor_test
pio run -e motor_test -t upload
pio device monitor -e motor_test
```

Use the serial monitor at **115200 baud**. Send one command character at a time;
Enter is optional and lowercase is accepted. Do not paste a sequence of moves.

| Command | Action |
|---|---|
| `H` or `?` | Show help |
| `F` / `B` | Forward / backward for one second |
| `L` / `R` | Spin left / right for one second |
| `A` / `C` | Left side only / right side only, forward for one second |
| `S` | Stop; driver goes into standby on next control tick |
| `1` … `5` | Select 10%, 20%, 30%, 40%, 50% duty while stopped |
| `E` | Clear an e-stop latch after the physical switch is released; stay stopped |

Start with wheels raised, send `A`, wait until stopped, then send `C`, then
`F`. Default duty is 20%. If the motors cannot start, stop and select `3` for
30% before retrying. The configured `prototype::kMaxDuty` cap still applies,
even if the selected percentage is higher. Swap one motor's two leads if its
wheel rotates backward relative to the other wheel on its side.

There is no automatic boot movement, repeating sequence, Pi protocol or
300 ms heartbeat requirement in this environment. Every move has a fixed
one-second deadline; new moves/speed changes are ignored while moving.
A dedicated 5 ms control task stops the motor outputs even if USB disconnects
or serial output blocks. Stopping the drive does not guarantee an immediate
mechanical stop; allow the wheels to stop before reversing.

GPIO 19 retains the active-low e-stop sense input. Pressing it latches the
stop until `E`; the physical e-stop must also interrupt motor power. An
unconnected GPIO 19 reads released, so this does not detect broken wiring.
No ToF, bump switch, encoder or drum is used in this test.

The normal firmware remains the default environment. Restore it with:

```sh
pio run -e esp32dev -t upload
```

Host checks for timed stops, e-stop recovery and timer wraparound:

```sh
c++ -std=c++17 -Iinclude tests/host/motor_test_state_test.cpp -o /tmp/fod-motor-test-state
/tmp/fod-motor-test-state
```

## Direct GPIO diagnostic — when the motor does not move

`motor_gpio_test` is a simpler diagnostic than `motor_test`: it uses only
`digitalWrite`, with no LEDC PWM, motor wrapper, PID, Pi protocol or enable
handshake. Each command applies **full motor-supply voltage** for one second,
then disables STBY. This deliberately bypasses `prototype::kMaxDuty`; use only
with the intended approximately 6 V motor supply and wheels raised.

```sh
pio run -e motor_gpio_test -t upload
pio device monitor -e motor_gpio_test
```

Close any existing monitor before upload. Select this environment explicitly:
plain `pio run -t upload` still uploads the normal `esp32dev` firmware, which
does not respond to these commands. Connect computer USB directly to ESP32.

Send `H` (Enter optional) and confirm a `GPIO TEST` reply. Then send `A` for
left wheels only or `C` for right wheels only. `F` runs both forward, `B` both
backward, and `S` stops. Lowercase works. A `RUN` reply confirms the software
command was accepted, not that electrical power reached the motor. Every run
ends after one second with `AUTO STOP`. Send individual commands, not a pasted
sequence. GPIO19 e-stop remains active and latches; `E` clears it after release.
An unconnected sense pin reads released. Serial messages can be dropped if the
output buffer is full, so printing cannot block the timed stop.

If nothing moves:

1. No `GPIO TEST` reply to `H`: check selected environment, upload success,
   USB data cable, serial port and 115200 baud. Press ESP32 reset to see the
   startup banner. Garbled output or repeated resets is a separate clue.
2. `BLOCKED`: GPIO19 is low or the e-stop is latched. Check the sense wiring,
   release the physical switch, and send `E`.
3. `RUN A` but no motion: measure at the driver board. VCC should be 3.3 V;
   VM should be near the AA pack voltage, including during startup. Grounds
   must be joined. During the one-second A pulse, STBY and PWMA should be
   near 3.3 V. With the default left sign, AIN1 is high and AIN2 low.
4. Control inputs correct: measure voltage **across AO1 and AO2**, not just
   one motor terminal to ground. Missing output suggests supply, driver or
   connection trouble; voltage present but no rotation suggests a motor,
   output-wire, mechanical-load or supply-current problem. A short pulse
   may be difficult for a slow meter to capture.

With power off, disconnect extra motors for a one-motor/channel test if needed;
this reduces battery loading. Never move wiring while powered. Motor power
comes from the AA pack into VM, not from the ESP32 USB or 3V3 pin.
