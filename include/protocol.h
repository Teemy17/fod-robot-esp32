#pragma once
// The ESP32 side of docs/protocol.md v1. Read that document (in
// fod-robot-pathing/docs/protocol.md) before touching this file -- it is a
// contract with the Pi's src/fodnav/link/esp32.py, and the Pi's tests
// (tests/test_esp32.py) encode exactly what it expects to receive.
//
// Framing: newline-terminated ASCII, single-space separated fields, at most
// 120 bytes including the terminator, at most 3 decimal places, no exponent
// notation. A malformed recognised command is dropped WHOLE, never acted on
// in part. An unrecognised first token is ignored, not an error -- that is
// what lets the Pi's boot-time line noise and this firmware's own boot spew
// coexist on the wire.

#include <Arduino.h>

constexpr uint8_t kProtoVersion = 1;
constexpr size_t kMaxLineBytes = 120;
constexpr uint32_t kWatchdogTimeoutMs = 300;  // protocol.md section 6

// Status flags, protocol.md section 4. Keep in bit-for-bit sync with
// fodnav.link.esp32.Flags.
enum StatusFlag : uint8_t {
  kFlagMotorsEnabled = 0x01,
  kFlagWatchdogFired  = 0x02,
  kFlagDriverFault    = 0x04,
  kFlagObstacle       = 0x08,
  kFlagEncoderFault   = 0x10,
  kFlagClamped        = 0x20,
  kFlagBatteryLow     = 0x40,
  kFlagFaultState     = 0x80,
};

enum class CmdType {
  kNone,        // unrecognised first token -- ignore, do not error
  kVelocity,    // V <v> <omega>
  kStop,        // S
  kEnable,      // E
  kDisable,     // D
  kMagnet,      // M <0|1>
  kTelemetryReq,// ?
  kHandshake,   // H
  kMalformed,   // recognised tag, bad field(s) -- drop the whole line
};

struct ParsedCommand {
  CmdType type = CmdType::kNone;
  float v = 0.0f;
  float omega = 0.0f;
  bool magnet_on = false;
};

// Does this command tag feed the watchdog? protocol.md section 6: exactly
// V/S/E/D/?. `M` is deliberately absent -- a drum command must not be able
// to mask a control loop that has stopped sending velocities.
bool feedsWatchdog(CmdType type);

// Parse one line (no trailing \n; \r already stripped by the caller's
// line reader). Returns kMalformed for a recognised-but-bad-field line and
// kNone for anything the firmware does not recognise -- callers must not act
// on either as if they were a real command.
ParsedCommand parseCommand(const String& line);

// --- device -> host lines -----------------------------------------------

// T <seq> <t_ms> <ticks_l> <ticks_r> <v_meas> <omega_meas> <flags(hex)>
void sendTelemetry(Stream& out, uint16_t seq, uint32_t t_ms, int32_t ticks_l,
                    int32_t ticks_r, float v_meas, float omega_meas,
                    uint8_t flags);

// I <proto> <fw_version> <ticks_per_rev> <wheel_radius_mm> <track_width_mm>
// Millimetres, not metres -- protocol.md section 5: 3 decimal places of a
// millimetre is 1e-6 m, which is exactly the tolerance the Pi's handshake
// asserts to. Do not "simplify" this to metres.
void sendInfo(Stream& out, float ticks_per_rev, float wheel_radius_m,
              float track_width_m, const char* fw_version);

// L <level> <message>  -- level is one of D/I/W/E. For humans; the Pi never
// parses state out of these.
void sendLog(Stream& out, char level, const String& message);
