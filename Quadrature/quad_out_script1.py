#!/usr/bin/env python3
"""
Author: Alex White, 2/8/2026
This is a simple testing script that emits simulated quadrature output and angle values to stdout.
Can be used along with `output_display.py` to visualize the output, or simply on its own to verify the 
quadrature signal generation and performance of the system under different angle inputs.
Configuration: set NUM_AXES to 1 or 2
  1 axis:  A,B,Index,Angle
  2 axes:  A1,B1,A2,B2,Index,Angle1,Angle2
Runs indefinitely until killed.
"""
import time
import math
import sys

NUM_AXES = 1  # set to 1 or 2 for single or dual axis mode
CPR = 4096  # Cycles Per Revolution
ANGLE_RESOLUTION = 360.0 / (4 * CPR)  # minimum angle step (0.0220 degrees)

def quantize_angle_to_cpr(angle_deg):
    """Quantize angle to the nearest CPR resolution step."""
    return round(angle_deg / ANGLE_RESOLUTION) * ANGLE_RESOLUTION

def quadrature_pattern_from_angle(angle_deg):
    """Map angle to quadrature state using Gray code pattern.
    With CPR=4096, this cycles through 4 states per complete revolution.
    """
    # Map angle [0,360) to position within one cycle [0,4*CPR)
    position = int((angle_deg % 360.0) / 360.0 * (4 * CPR)) % (4 * CPR)
    # Map position to quadrature state (0-3)
    state = position % 4
    # Gray code quadrature pattern: (A, B)
    patterns = [(0, 0), (1, 0), (1, 1), (0, 1)]
    return patterns[state]

def main():
    output_rate = 0.05  # 20 Hz output (for display responsiveness)
    iteration = 0
    try:
        while True:
            # Calculate time based on iteration count and output rate
            t = iteration * output_rate
            
            # simulated angles (deg) - sweeping motion to show all quadrature states
            raw_angle1 = (t * 1.0) % 360.0  # steady rotation at 1 deg/sec
            raw_angle2 = ((t * 10.0) + 90.0) % 360.0  # steady rotation at 10 deg/sec, offset 90 degrees
            
            # Quantize to CPR resolution and wrap to [0, 360)
            angle1 = quantize_angle_to_cpr(raw_angle1) % 360.0
            angle2 = quantize_angle_to_cpr(raw_angle2) % 360.0

            a1, b1 = quadrature_pattern_from_angle(angle1)
            index = 1 if (abs(angle1) < 0.01 or abs(angle1 - 360.0) < 0.01) else 0

            if NUM_AXES == 1:
                # single axis: A,B,Index,Angle
                line = f"{a1},{b1},{index},{angle1:.4f}\n"
            else:
                # dual axis: A1,B1,A2,B2,Index,Angle1,Angle2
                a2, b2 = quadrature_pattern_from_angle(angle2)
                line = f"{a1},{b1},{a2},{b2},{index},{angle1:.4f},{angle2:.4f}\n"

            sys.stdout.write(line)
            sys.stdout.flush()
            time.sleep(output_rate)  # synchronized with angle update rate
            iteration += 1
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

