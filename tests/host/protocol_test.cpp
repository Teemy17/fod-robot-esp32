#include <cassert>
#include "protocol.h"
int main() {
  auto p = parseCommand("P -0.250 0.500");
  assert(p.type == CmdType::kPwm && p.v == -0.25f && p.omega == 0.5f);
  assert(feedsWatchdog(p.type));
  for (const char* bad : {"P nan 0", "P inf 0", "P 1e2 0", "P +1 0",
                         "P 0.0001 0", "P - 0", "P . 0", "P 0", "P 0 0 1",
                         "P  0 0", "E 1"}) {
    assert(parseCommand(bad).type == CmdType::kMalformed);
  }
  assert(parseCommand("P 2 -2").type == CmdType::kPwm); // control clamps
  assert(parseCommand("V 0.1 0").type == CmdType::kVelocity);
  assert(!feedsWatchdog(CmdType::kVelocity));
  for (const char* removed : {"M 1", "M 0", "DRUM on"})
    assert(parseCommand(removed).type == CmdType::kNone);
  assert(!feedsWatchdog(CmdType::kNone));
  assert(!feedsWatchdog(CmdType::kHandshake));
  assert(!feedsWatchdog(CmdType::kMalformed));
  for (const char* live : {"S", "E", "D", "?"})
    assert(feedsWatchdog(parseCommand(live).type));
  assert(kProtoVersion == 2);
}
