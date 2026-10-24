#pragma once
// Protocol v1: the dialect fod-robot-pathing (nav) speaks. V/S/E/D/M/?/H in, T/I/L out.
// This is separate from protocol.h on purpose: protocol.h is the v2 bench-prototype
// dialect (P/PT, kProtoVersion == 2) and its host tests and the harness's sim-v2 pin
// that behaviour. Shared constants (kMaxLineBytes, kWatchdogTimeoutMs, StatusFlag) and
// sendLog() come from protocol.h.

#include <Arduino.h>
#include "protocol.h"

namespace v1 {

constexpr uint8_t kProtoVersionV1 = 1;

enum class CmdType {
  kNone,          // unrecognised first token -- ignore, do not error
  kVelocity,      // V <v m/s> <omega rad/s>
  kStop,          // S
  kEnable,        // E
  kDisable,       // D
  kMagnet,        // M 0|1
  kTelemetryReq,  // ?
  kHandshake,     // H
  kMalformed,     // recognised tag, bad field(s) -- drop the whole line
};

struct ParsedCommand {
  CmdType type = CmdType::kNone;
  float v = 0.0f;
  float omega = 0.0f;
  bool magnetOn = false;
};

// Exactly V/S/E/D/? feed the watchdog. H and M do not, and neither do unknown or
// malformed lines.
bool feedsWatchdog(CmdType type);

// Parse one line (no trailing \n). Numbers: finite decimals, at most three decimal
// places, no exponent, no leading '+'. P (the v2 command) is NOT recognised here.
ParsedCommand parseCommand(const String& line);

// Whole lines, newline included, so a caller can write each with ONE Serial write.
// T <seq> <t_ms> <ticks_l> <ticks_r> <v_meas> <omega_meas> <flags(hex)>
String formatTelemetry(uint16_t seq, uint32_t t_ms, int32_t ticks_l, int32_t ticks_r,
                       float v_meas, float omega_meas, uint8_t flags);

// I 1 <fw_version> <ticks_per_rev> <wheel_radius_mm> <track_width_mm>
// Millimetres, not metres: 3 decimals of a millimetre is the 1e-6 m tolerance the Pi's
// handshake asserts to.
String formatInfo(const char* fw_version, float ticks_per_rev, float wheel_radius_m,
                  float track_width_m);

}  // namespace v1
