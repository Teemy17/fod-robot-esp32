#!/usr/bin/env python3
"""Timed, 50 Hz TT motor bench test. Requires pyserial; opens real hardware."""
import argparse
import math
import time
import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--left', type=float, default=0.2)
    parser.add_argument('--right', type=float, default=0.2)
    parser.add_argument('--seconds', type=float, default=1.0)
    args = parser.parse_args()
    if not all(math.isfinite(x) for x in (args.left, args.right, args.seconds)):
        parser.error('arguments must be finite')
    if max(abs(args.left), abs(args.right)) > 0.5 or not 0 < args.seconds <= 10:
        parser.error('duties must be within +/-0.5; duration must be (0, 10] seconds')
    with serial.Serial(args.port, 115200, timeout=0.1, write_timeout=0.1) as link:
        time.sleep(2)
        link.reset_input_buffer()
        link.write(b'\nH\n')
        deadline = time.monotonic() + 2
        verified = False
        while time.monotonic() < deadline:
            if link.readline().strip() == b'I 2 tt-prototype-0.2.0 0.000 0.000 0.000':
                verified = True
                break
        if not verified:
            raise RuntimeError('Expected TT prototype firmware handshake; motors not enabled')
        try:
            link.write(b'E\n')
            end = time.monotonic() + args.seconds
            while time.monotonic() < end:
                link.write(f'P {args.left:.3f} {args.right:.3f}\n'.encode('ascii'))
                if link.in_waiting:
                    link.read(link.in_waiting)
                time.sleep(0.02)
        finally:
            link.write(b'S\nD\n')
            link.flush()


if __name__ == '__main__':
    main()
