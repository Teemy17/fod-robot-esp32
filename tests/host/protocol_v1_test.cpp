#include <cassert>
#include <cstring>
#include "protocol_v1.h"

using Cmd = v1::CmdType;  // protocol.h (v2) also declares a global CmdType

int main() {
  // --- V ---
  auto v = v1::parseCommand("V 0.250 -0.400");
  assert(v.type == Cmd::kVelocity && v.v == 0.25f && v.omega == -0.4f);
  assert(v1::parseCommand("V 0 0").type == Cmd::kVelocity);
  for (const char* bad : {"V", "V 0.25", "V 0.25 0 0", "V nan 0", "V inf 0", "V 1e2 0",
                          "V +1 0", "V 0.0001 0", "V - 0", "V . 0", "V  0 0", "V 0 abc"}) {
    assert(v1::parseCommand(bad).type == Cmd::kMalformed);
  }

  // --- M ---
  auto on = v1::parseCommand("M 1");
  auto off = v1::parseCommand("M 0");
  assert(on.type == Cmd::kMagnet && on.magnetOn);
  assert(off.type == Cmd::kMagnet && !off.magnetOn);
  for (const char* bad : {"M", "M 2", "M 01", "M 1 1", "M on"})
    assert(v1::parseCommand(bad).type == Cmd::kMalformed);

  // --- single-token commands ---
  assert(v1::parseCommand("S").type == Cmd::kStop);
  assert(v1::parseCommand("E").type == Cmd::kEnable);
  assert(v1::parseCommand("D").type == Cmd::kDisable);
  assert(v1::parseCommand("?").type == Cmd::kTelemetryReq);
  assert(v1::parseCommand("H").type == Cmd::kHandshake);
  for (const char* bad : {"S 1", "E 1", "D x", "? 1", "H 1"})
    assert(v1::parseCommand(bad).type == Cmd::kMalformed);

  // --- ignored, never acted on ---
  for (const char* ignored : {"", "   ", "Q 1", "P 0.2 0.2", "DRUM on", "v 0 0"})
    assert(v1::parseCommand(ignored).type == Cmd::kNone);  // P is the v2 command
  assert(v1::parseCommand("V 0.100 0.000\r").type == Cmd::kVelocity);  // stray CR

  // --- watchdog: exactly V/S/E/D/? ---
  for (Cmd t : {Cmd::kVelocity, Cmd::kStop, Cmd::kEnable,
                    Cmd::kDisable, Cmd::kTelemetryReq})
    assert(v1::feedsWatchdog(t));
  for (Cmd t : {Cmd::kNone, Cmd::kMagnet, Cmd::kHandshake,
                    Cmd::kMalformed})
    assert(!v1::feedsWatchdog(t));

  // --- wire format ---
  assert(std::strcmp(v1::formatTelemetry(7, 1234, -100, 200, 0.1234f, -0.4567f, 0x21).c_str(),
                     "T 7 1234 -100 200 0.123 -0.457 21\n") == 0);
  assert(std::strcmp(v1::formatTelemetry(65535, 4294967295u, -2147483647 - 1, 2147483647,
                                         -0.0001f, 0.0f, 0xff).c_str(),
                     "T 65535 4294967295 -2147483648 2147483647 0.000 0.000 ff\n") == 0);
  assert(std::strcmp(v1::formatInfo("esp32-fod-1.0.0-dev", 4680.0f, 0.0325f, 0.2f).c_str(),
                     "I 1 esp32-fod-1.0.0-dev 4680.000 32.500 200.000\n") == 0);
  // unmeasured: the handshake reports zeros so nav refuses
  assert(std::strcmp(v1::formatInfo("fw", 0.0f, 0.0f, 0.0f).c_str(),
                     "I 1 fw 0.000 0.000 0.000\n") == 0);
  // worst-case lines stay inside the 120-byte framing limit
  assert(std::strlen(v1::formatTelemetry(65535, 4294967295u, -2147483647 - 1, -2147483647,
                                         -99999.999f, -99999.999f, 0xff).c_str()) < kMaxLineBytes);

  std::puts("protocol_v1_test: ok");
}
