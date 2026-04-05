// quadrature_decoder.c
// Authors: Alex White, Richard Harrison
// Date: 2026-03-24
// Version: 1
//
// This file contains the functionality of the quadrature decoding for the S26-14 project.
// Intended functionality: The quadrature output is the chosen method of outputting angle data from the sensors on the device. This method
// of output is not easily read by humans, and so a decoder is needed to take the quadrature output and convert it into a human-readable angle value.
// The angle range is 0 to 360. Mostly intended for use in testing the quadrature encoded output.

#ifndef QUADRATURE_COMMON_H
#define QUADRATURE_COMMON_H

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define MAX_AXES 2
#define QUADRATURE_STATES 4

#define MIN_CPR 1
#define MAX_CPR 9000
#define DEGREES_PER_REVOLUTION 360.0

#define TOLERANCE 0.001
#define ANGLE_TOLERANCE TOLERANCE
#define RADIAN_TOLERANCE TOLERANCE

#endif
