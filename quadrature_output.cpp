//QUADRATURE OUTPUT
//
//Author: Alex White, 2/2/2026
//Version: 1
//
//This file contains the functionality of the quadrature encoded output for the S26-14 project.
//Intended functionality: The quadrature output will serve as the method of relying the positional data from the sensors on the PCB
//to the robot via two output pins. The quadrature resolution may be set to any value from 1 to 9000 CPR (cycles per revolution) 
//corresponding to 4 to 36000 positions per revolution. When the resolution is set to 9000 CPR, one quadrature state change on the 
//A/B outputs corresponds to 0.01 degree. The PCB calculates the absolute position at a 640 Hz rate and updates the A/B signals 
//in bursts of pulses at an average rate of approximately 6900 quadrature states per second. It is important to note that the
//quadrature signal itself does not necessarily indicate what the absolute angle is, simply the change in angle from a previous
//value. Therefore, it is very important that the calibration procedure functions properly and the inclinometer knows which angle it
//is starting from. Intended to be used in conjuction with the quadrature decoder functionality for proper testing and verification.


#include "quadrature_output.hpp"
