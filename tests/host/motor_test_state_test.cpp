#include <cassert>
#include "motor_test_state.h"

int main() {
  MotorTestState s;
  assert(!s.moving());
  assert(s.command('F', 100, false));
  assert(s.left == 20 && s.right == 20);
  assert(!s.command('B', 500, false)); // no reversal or deadline extension
  s.tick(1099, false);
  assert(s.moving());
  s.tick(1100, false);
  assert(!s.moving());
  assert(s.command('5', 1200, false));
  assert(s.command('L', 1200, false));
  assert(s.left == -50 && s.right == 50);
  s.tick(1201, true);
  assert(!s.moving() && s.fault);
  assert(!s.command('E', 1202, true));
  assert(!s.command('F', 1203, false));
  assert(s.command('E', 1204, false));
  assert(!s.moving());
  assert(s.command('A', UINT32_MAX - 499, false));
  assert(s.left == 50 && s.right == 0);
  s.tick(499, false);
  assert(s.moving());
  s.tick(500, false);
  assert(!s.moving());
  assert(!s.command('X', 501, false));
  assert(s.command('C', 502, false));
  assert(s.left == 0 && s.right == 50);
  assert(s.command('S', 503, false));
  assert(!s.moving());
}
