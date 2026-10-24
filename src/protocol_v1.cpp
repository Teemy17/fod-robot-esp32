#include "protocol_v1.h"

namespace {

// Single-space split. protocol.md fields have no repeated spaces.
int splitFields(const String& line, String out[], int maxFields) {
  int n = 0;
  int start = 0;
  while (n < maxFields) {
    int sp = line.indexOf(' ', start);
    if (sp < 0) {
      out[n++] = line.substring(start);
      break;
    }
    out[n++] = line.substring(start, sp);
    start = sp + 1;
  }
  return n;
}

// Same strictness as protocol.cpp (the v2 codec): optional '-', digits, optional '.',
// at most three decimals. Rejects '+', exponents, nan/inf.
bool parseFloatStrict(const String& tok, float& value) {
  if (tok.length() == 0) return false;
  size_t i = tok[0] == '-' ? 1 : 0;
  bool digit = false, dot = false;
  int decimals = 0;
  for (; i < tok.length(); ++i) {
    char c = tok[i];
    if (c == '.' && !dot) { dot = true; continue; }
    if (c < '0' || c > '9') return false;
    digit = true;
    if (dot && ++decimals > 3) return false;
  }
  if (!digit) return false;
  char* end = nullptr;
  value = strtof(tok.c_str(), &end);
  return end != nullptr && *end == '\0' && isfinite(value);
}

// 3 decimals, no exponent, negative zero normalised away -- mirrors
// fodnav.link.esp32._fmt.
String fmt3(float value) {
  if (fabsf(value) < 0.0005f) value = 0.0f;  // avoid "-0.000"
  char buf[16];
  dtostrf(value, 0, 3, buf);
  return String(buf);
}

}  // namespace

namespace v1 {

bool feedsWatchdog(CmdType type) {
  switch (type) {
    case CmdType::kVelocity:
    case CmdType::kStop:
    case CmdType::kEnable:
    case CmdType::kDisable:
    case CmdType::kTelemetryReq:
      return true;
    default:
      return false;
  }
}

ParsedCommand parseCommand(const String& raw) {
  ParsedCommand cmd;
  String line = raw;
  line.trim();
  if (line.length() == 0) return cmd;  // kNone

  String fields[4];
  int n = splitFields(line, fields, 4);
  const String& tag = fields[0];

  if (tag == "V") {
    float v, omega;
    if (n != 3 || !parseFloatStrict(fields[1], v) || !parseFloatStrict(fields[2], omega)) {
      cmd.type = CmdType::kMalformed;
      return cmd;
    }
    cmd.type = CmdType::kVelocity;
    cmd.v = v;
    cmd.omega = omega;
    return cmd;
  }
  if (tag == "M") {
    if (n != 2 || (!(fields[1] == "0") && !(fields[1] == "1"))) {
      cmd.type = CmdType::kMalformed;
      return cmd;
    }
    cmd.type = CmdType::kMagnet;
    cmd.magnetOn = (fields[1] == "1");
    return cmd;
  }
  if (tag == "S" || tag == "E" || tag == "D" || tag == "?" || tag == "H") {
    if (n != 1) { cmd.type = CmdType::kMalformed; return cmd; }
    if (tag == "S") cmd.type = CmdType::kStop;
    else if (tag == "E") cmd.type = CmdType::kEnable;
    else if (tag == "D") cmd.type = CmdType::kDisable;
    else if (tag == "?") cmd.type = CmdType::kTelemetryReq;
    else cmd.type = CmdType::kHandshake;
    return cmd;
  }

  // Unrecognised first token (including the v2 "P"): not an error, ignored.
  return cmd;  // kNone
}

String formatTelemetry(uint16_t seq, uint32_t t_ms, int32_t ticks_l, int32_t ticks_r,
                       float v_meas, float omega_meas, uint8_t flags) {
  char buf[112];
  snprintf(buf, sizeof(buf), "T %u %lu %ld %ld %s %s %02x\n", (unsigned)seq,
           (unsigned long)t_ms, (long)ticks_l, (long)ticks_r, fmt3(v_meas).c_str(),
           fmt3(omega_meas).c_str(), (unsigned)flags);
  return String(buf);
}

String formatInfo(const char* fw_version, float ticks_per_rev, float wheel_radius_m,
                  float track_width_m) {
  char buf[112];
  snprintf(buf, sizeof(buf), "I %u %s %s %s %s\n", (unsigned)kProtoVersionV1, fw_version,
           fmt3(ticks_per_rev).c_str(), fmt3(wheel_radius_m * 1000.0f).c_str(),
           fmt3(track_width_m * 1000.0f).c_str());
  return String(buf);
}

}  // namespace v1
