"""Author: Alex White
Basic test script for live angle output to output_display.py.
Simulates one or two axes rotating at configurable speeds.

Writes angle lines to the shared temp file read by output_display.py --live.
Run this script first, then launch the display in a separate terminal:
  python live_angle_test.py
  python output_display.py --live

Dual axis:
  python live_angle_test.py --axes 2
  python output_display.py --live --axes 2

Options:
  --axes 2        Enable dual axis output
  --speed1 <f>    Degrees per step for axis 1 (default: 1.0)
  --speed2 <f>    Degrees per step for axis 2 (default: -1.5)
  --rate <ms>     Output rate in milliseconds (default: 100)
"""
import sys
import os
import time
import tempfile


# Must match LIVE_DATA_FILE in output_display.py
LIVE_DATA_FILE = os.path.join(tempfile.gettempdir(), 'quad_live_angles.txt')

NUM_AXES = 1
SPEED1   = 1.0
SPEED2   = -1.5
RATE_MS  = 100


def parse_args():
    global NUM_AXES, SPEED1, SPEED2, RATE_MS
    args = sys.argv[1:]
    if '--axes'   in args:
        try: NUM_AXES = int(args[args.index('--axes') + 1])
        except (IndexError, ValueError): pass
    if '--speed1' in args:
        try: SPEED1 = float(args[args.index('--speed1') + 1])
        except (IndexError, ValueError): pass
    if '--speed2' in args:
        try: SPEED2 = float(args[args.index('--speed2') + 1])
        except (IndexError, ValueError): pass
    if '--rate'   in args:
        try: RATE_MS = int(args[args.index('--rate') + 1])
        except (IndexError, ValueError): pass


def main():
    parse_args()
    interval = RATE_MS / 1000.0
    angle1 = 0.0
    angle2 = 0.0

    print(f'Writing to: {LIVE_DATA_FILE}')
    print('Start output_display.py --live in another terminal.')
    print('Ctrl+C to stop.\n')

    while True:
        line = f'{angle1:.3f}\n' if NUM_AXES == 1 else f'{angle1:.3f},{angle2:.3f}\n'
        with open(LIVE_DATA_FILE, 'a') as f:
            f.write(line)

        angle1 = (angle1 + SPEED1) % 360
        angle2 = (angle2 + SPEED2) % 360
        time.sleep(interval)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nStopped.')