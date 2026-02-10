#!/usr/bin/env python3
"""
Simple testing script that emits simulated quadrature output and angle values to stdout.
Configuration: set NUM_AXES to 1 or 2
  1 axis:  A,B,Index,Angle
  2 axes:  A1,B1,A2,B2,Index,Angle1,Angle2
Runs indefinitely until killed.
"""
import time
import math
import sys

NUM_AXES = 2  # set to 1 or 2 for single or dual axis mode

def quadrature_pattern_from_angle(angle_deg):
    # map angle [0,360) to one of 4 quadrature states
    state = int((angle_deg % 360) / 360.0 * 4) % 4
    if state == 0:
        return 0,0
    if state == 1:
        return 1,0
    if state == 2:
        return 1,1
    return 0,1

def main():
    t0 = time.time()
    try:
        while True:
            t = time.time() - t0
            # simulated angles (deg)
            angle1 = (180.0 + 90.0 * math.sin(0.5 * t)) % 360.0
            angle2 = (90.0 + 60.0 * math.sin(0.9 * t + 1.0)) % 360.0

            a1, b1 = quadrature_pattern_from_angle(angle1)
            index = 1 if (abs(angle1) < 0.01 or abs(angle1 - 360.0) < 0.01) else 0

            if NUM_AXES == 1:
                # single axis: A,B,Index,Angle
                line = f"{a1},{b1},{index},{angle1:.3f}\n"
            else:
                # dual axis: A1,B1,A2,B2,Index,Angle1,Angle2
                a2, b2 = quadrature_pattern_from_angle(angle2)
                line = f"{a1},{b1},{a2},{b2},{index},{angle1:.3f},{angle2:.3f}\n"

            sys.stdout.write(line)
            sys.stdout.flush()
            time.sleep(0.02)  # 50 Hz
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
