#!/usr/bin/env python3
"""Timed, 50 Hz bench drive for the nav_v1 firmware (protocol v1: V/T/I).

Sends `V <v> <omega>` at 50 Hz for a few seconds and prints what the board reports, so
you can check motor direction, encoder sign, and the wheel-speed loop with the wheels
raised. Requires pyserial. Opens real hardware.

Unlike tt_drive.py (the v2 bench prototype), this checks for an `I 1 ...` handshake. The
handshake's constants are 0.000 until the robot is measured (see include/robot_v1.h);
that is expected here and does not stop this script.
"""
import argparse
import math
import time
import serial

V_LIMIT = 0.25      # m/s, matches robot::kVMaxMps
OMEGA_LIMIT = 1.5   # rad/s, matches robot::kOmegaMaxRadps


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', required=True)
    parser.add_argument('--v', type=float, default=0.10, help='forward speed, m/s')
    parser.add_argument('--omega', type=float, default=0.0, help='turn rate, rad/s (CCW +)')
    parser.add_argument('--seconds', type=float, default=3.0)
    parser.add_argument('--every', type=int, default=5,
                        help='print every Nth T line (50 Hz / N)')
    args = parser.parse_args()
    if not all(math.isfinite(x) for x in (args.v, args.omega, args.seconds)):
        parser.error('arguments must be finite')
    if abs(args.v) > V_LIMIT or abs(args.omega) > OMEGA_LIMIT or not 0 < args.seconds <= 10:
        parser.error(f'|v| <= {V_LIMIT}, |omega| <= {OMEGA_LIMIT}, 0 < seconds <= 10')
    if args.every < 1:
        parser.error('--every must be >= 1')

    with serial.Serial(args.port, 115200, timeout=0.02, write_timeout=0.1) as link:
        time.sleep(2)  # the board resets when the port opens
        link.reset_input_buffer()
        link.write(b'\nH\n')
        deadline = time.monotonic() + 2
        info = None
        while time.monotonic() < deadline:
            line = link.readline().decode('ascii', 'replace').strip()
            if line.startswith('I '):
                info = line
                break
        if not info or not info.startswith('I 1 '):
            raise RuntimeError(f'Expected an "I 1 ..." handshake from the nav_v1 firmware, '
                               f'got {info!r}; motors not enabled')
        print('handshake:', info)

        seen = 0
        try:
            link.write(b'E\n')
            end = time.monotonic() + args.seconds
            command = f'V {args.v:.3f} {args.omega:.3f}\n'.encode('ascii')
            while time.monotonic() < end:
                link.write(command)  # must repeat: the watchdog trips after 300 ms
                for raw in link.read(link.in_waiting or 1).decode('ascii', 'replace').splitlines():
                    if raw.startswith('T '):
                        seen += 1
                        if seen % args.every == 0:
                            print(raw)
                    elif raw.startswith('L '):
                        print(raw)
                time.sleep(0.02)
        finally:
            link.write(b'S\nD\n')
            link.flush()


if __name__ == '__main__':
    main()
