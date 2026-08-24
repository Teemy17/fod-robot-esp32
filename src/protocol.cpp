#include "protocol.h"

namespace {

// Split on single spaces. protocol.md fields are single-space separated with
// no repeated spaces, so a plain split is enough -- no need to collapse runs.
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

bool parseFloatStrict(const String& tok, float& value) {
  if (tok.length() == 0) return false;
  // No exponent notation on the wire (protocol.md section 2).
  if (tok.indexOf('e') >= 0 || tok.indexOf('E') >= 0) return false;
  char* end = nullptr;
  value = strtod(tok.c_str(), &end);
  return end != nullptr && *end == '\0';
}

// 3 decimals, no exponent, negative zero normalised away -- mirrors
// fodnav.link.esp32._fmt exactly so both sides agree on what a number
// looks like on the wire.
String fmt3(float value) {
  if (fabsf(value) < 0.0005f) value = 0.0f;  // avoid "-0.000"
  char buf[16];
  dtostrf(value, 0, 3, buf);
  return String(buf);
}

}  // namespace

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
  line.trim();  // also strips a stray \r the line reader missed
  if (line.length() == 0) return cmd;  // kNone

  String fields[4];
  int n = splitFields(line, fields, 4);
  const String& tag = fields[0];

  if (tag == "V") {
    if (n != 3) { cmd.type = CmdType::kMalformed; return cmd; }
    float v, omega;
    if (!parseFloatStrict(fields[1], v) || !parseFloatStrict(fields[2], omega)) {
      cmd.type = CmdType::kMalformed;
      return cmd;
    }
    cmd.type = CmdType::kVelocity;
    cmd.v = v;
    cmd.omega = omega;
    return cmd;
  }
  if (tag == "M") {
    if (n != 2 || (fields[1] != "0" && fields[1] != "1")) {
      cmd.type = CmdType::kMalformed;
      return cmd;
    }
    cmd.type = CmdType::kMagnet;
    cmd.magnet_on = (fields[1] == "1");
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

  // Unrecognised first token: not an error. Boot spew, a future command a
  // newer Pi sends to an older firmware, etc.
  return cmd;  // kNone
}

void sendTelemetry(Stream& out, uint16_t seq, uint32_t t_ms, int32_t ticks_l,
                    int32_t ticks_r, float v_meas, float omega_meas,
                    uint8_t flags) {
  out.printf("T %u %lu %ld %ld %s %s %02x\n",
             (unsigned)seq, (unsigned long)t_ms, (long)ticks_l, (long)ticks_r,
             fmt3(v_meas).c_str(), fmt3(omega_meas).c_str(), flags);
}

void sendInfo(Stream& out, float ticks_per_rev, float wheel_radius_m,
              float track_width_m, const char* fw_version) {
  // Millimetres on the wire -- see protocol.h. 3 decimals of a millimetre is
  // exactly the 1e-6 m tolerance the Pi's handshake compares against.
  out.printf("I %u %s %s %s %s\n",
             kProtoVersion, fw_version,
             fmt3(ticks_per_rev).c_str(),
             fmt3(wheel_radius_m * 1000.0f).c_str(),
             fmt3(track_width_m * 1000.0f).c_str());
}

void sendLog(Stream& out, char level, const String& message) {
  out.print("L ");
  out.print(level);
  out.print(' ');
  out.println(message);
}
