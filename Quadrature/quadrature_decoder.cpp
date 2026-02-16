//Author: Alex White, 2/2/2026
//Version: 1
//
//This file contains the functionality of the quadrature decoding for the S26-14 project.
//Intended functionality: The quadrature output is the chosen method of outputting angle data from the sensors on the device. This method
//of output is not easily read by humans, and so a decoder is needed to take the quadrature output and convert it into a human-readable angle value.
//The angle range is 0 to 360. Mostly intended for use in testing the quadrature encoded output.


#include "quadrature_decoder.hpp"