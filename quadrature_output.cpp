//Author: Alex White, 2/2/2026
//Version: 1
//This file contains the functionality of the quadrature encoded output for the S26-14 project.
//Intended functionality: The quadrature output will serve as the method of relying the positional data from the sensors on the PCB
//to the robot via two output pins. With positive rotation, channel A will lead channel B by 90 degrees and vice versa for negative
//rotation. The quadrature resolution may be set to any value from 1 to 9000 CPR (cycles per revolution) 
//corresponding to 4 to 36000 positions per revolution. When the resolution is set to 9000 CPR, one quadrature state change on the 
//A/B outputs corresponds to 0.01 degree. The PCB calculates the absolute position at a 640 Hz rate and updates the A/B signals 
//in bursts of pulses at an average rate of approximately 6900 quadrature states per second. It is important to note that the
//quadrature signal itself does not necessarily indicate what the absolute angle is, simply the change in angle from a previous
//value. Therefore, it is very important that the calibration procedure functions properly and the inclinometer knows the angle it
//is starting from. The quadrature functionality does not know or care how the angle is calculated, simply that it is provided with
//an angle. Intended to be used in conjuction with the quadrature decoder function for proper testing and verification.


#include "quadrature_output.hpp"
//high and low signals
#define HIGH 1;
#define LOW 0;
//default cycles per revolution
#define CYCLES_PER_REV_DEFAULT 4096;
//absolute position calculated at 640 Hz
#define CALCULATION_RATE_HZ 640;
//quadrature states output at a rate of 6900 Hz
#define OUTPUT_RATE_HZ 6900;
//Cyles per revolution (chosen by programmer, default 4096). Corresponds to 4*n counts per revolution.
#define CYCLES_PER_REV CYCLES_PER_REV_DEFAULT;
//positions per revolution (4 * cycles per revolution)
#define POSITIONS_PER_REV 4*CYCLES_PER_REV;
//Maximum reportable rate of rotation in degrees per second, based on CPR
#define MAX_DEGREES_PER_SEC = 614100 / CYCLES_PER_REV;
//Period of the quadrature output signals
#define SIGNAL_PERIOD 
//Output channels, starting low. Index goes high only when the angle is 0 and both channels are low.
int CHANNEL_A_AXIS_1, CHANNEl_A_AXIS_2, CHANNEL_B_AXIS_1, CHANNEL_B_AXIS_2, INDEX_AXIS_1, INDEX_AXIS_2 = 0;
//flag for whether or not the inclinometer has been calibrated and the output should begin.
bool calibrated = false;
//starting angle (gotten after calibration)
double starting_angle=0.0;

//tentatively, GPIO pins 2-4 or 8-13