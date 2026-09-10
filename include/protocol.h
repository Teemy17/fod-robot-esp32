#pragma once
// Prototype serial dialect. See README for commands and compatibility.

#include <Arduino.h>

constexpr uint8_t kProtoVersion = 2;  // prototype dialect; see README
constexpr size_t kMaxLineBytes = 120;
constexpr uint32_t kWatchdogTimeoutMs = 300;  // protocol.md section 6

// Legacy v1 flags, retained for future encoder support; unused by PT.
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
  kPwm,         // P <left duty> <right duty> (normalized)
  kTelemetryReq,// ?
  kHandshake,   // H
  kMalformed,   // recognised tag, bad field(s) -- drop the whole line
};

struct ParsedCommand {
  CmdType type = CmdType::kNone;
  float v = 0.0f;      // left normalized duty for P
  float omega = 0.0f;  // right normalized duty for P
};

// Does this prototype command feed the watchdog? Exactly
// P/S/E/D/?. Unsupported commands never feed the watchdog.
bool feedsWatchdog(CmdType type);

// Parse one line (no trailing \n; \r already stripped by the caller's
// line reader). Returns kMalformed for a recognised-but-bad-field line and
// kNone for anything the firmware does not recognise -- callers must not act
// on either as if they were a real command.
ParsedCommand parseCommand(const String& line);

// --- device -> host lines -----------------------------------------------

// Legacy encoder codec (not emitted by this prototype).
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
