//Author: Alex White, 2/2/2026
//Version: 1
//
//This file is the header file for the quadrature output functionality. See other file for description.
#ifndef QUADRATURE_OUTPUT_HPP
#define QUADRATURE_OUTPUT_HPP

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <charconv>

//need to add: starting procedure, updating procedure, changing parameters, updating depending on precision, setting index to 0,
//tracking in case the device moves faster than we are updating, 

//stub for setting index to 0 (only when A, B are low and angle is 0)
void set_index(int A_1, int A_2, int B_1, int B_2, double angle);




#endif