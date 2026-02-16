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
#include <cmath>
#include <algorithm>

//Constants
const double DEGREES_PER_REVOLUTION = 360.0;
const int MAX_CPR = 9000;
const int MIN_CPR = 1;
const double ANGLE_TOLERANCE = 0.001;  //tolerance for floating point comparisons

//============================================================================================
//                          AxisState Implementation
//============================================================================================

//AxisState Constructor
QuadratureOutput::AxisState::AxisState(int cpr_val)
    : cpr(std::max(MIN_CPR, std::min(cpr_val, MAX_CPR))),
      positions_per_rev(4 * cpr),
      starting_angle(0.0),
      previous_angle(0.0),
      position_count(0),
      channel_a(0),
      channel_b(0),
      index(0),
      calibrated(false) {
}

//AxisState: Convert angle difference to position change
int QuadratureOutput::AxisState::angleToPositionChange(double angle_diff) {
    double raw_position_change = (angle_diff / DEGREES_PER_REVOLUTION) * positions_per_rev;
    return static_cast<int>(std::round(raw_position_change));
}

//AxisState: Get the quadrature pattern for a position
void QuadratureOutput::AxisState::getQuadraturePattern(int position, int& a, int& b) const {
    int state = position % 4;
    switch (state) {
        case 0:
            a = 0; b = 0;
            break;
        case 1:
            a = 1; b = 0;
            break;
        case 2:
            a = 1; b = 1;
            break;
        case 3:
            a = 0; b = 1;
            break;
        default:
            a = 0; b = 0;
    }
}

//AxisState: Update quadrature states based on position change
void QuadratureOutput::AxisState::updateQuadratureStates(int position_change) {
    if (position_change == 0) {
        return;
    }
    getQuadraturePattern(position_count, channel_a, channel_b);
}

//============================================================================================
//                          QuadratureOutput Main Implementation
//============================================================================================

//Constructor
QuadratureOutput::QuadratureOutput(int cpr, int num_axes_init)
    : axis1(cpr), axis2(cpr), num_axes(std::max(1, std::min(num_axes_init, 2))) {
}

//Initialize both axes with starting angles
void QuadratureOutput::initialize(double initial_angle_axis1, double initial_angle_axis2) {
    //Initialize axis 1
    axis1.starting_angle = initial_angle_axis1;
    axis1.previous_angle = initial_angle_axis1;
    axis1.position_count = 0;
    axis1.calibrated = true;
    axis1.channel_a = 0;
    axis1.channel_b = 0;
    axis1.index = 0;
    if (std::abs(initial_angle_axis1) < ANGLE_TOLERANCE || std::abs(initial_angle_axis1 - 360.0) < ANGLE_TOLERANCE) {
        axis1.index = 1;
    }
    
    //Initialize axis 2 only if num_axes == 2
    if (num_axes == 2) {
        axis2.starting_angle = initial_angle_axis2;
        axis2.previous_angle = initial_angle_axis2;
        axis2.position_count = 0;
        axis2.calibrated = true;
        axis2.channel_a = 0;
        axis2.channel_b = 0;
        axis2.index = 0;
        if (std::abs(initial_angle_axis2) < ANGLE_TOLERANCE || std::abs(initial_angle_axis2 - 360.0) < ANGLE_TOLERANCE) {
            axis2.index = 1;
        }
    }
}

//Update both axes with new angle readings
void QuadratureOutput::update(double current_angle_axis1, double current_angle_axis2) {
    if (!axis1.calibrated || (num_axes == 2 && !axis2.calibrated)) {
        return;
    }
    
    //Update axis 1 (always)
    {
        double angle_diff = current_angle_axis1 - axis1.previous_angle;
        while (angle_diff > 180.0) angle_diff -= 360.0;
        while (angle_diff < -180.0) angle_diff += 360.0;
        
        int position_change = axis1.angleToPositionChange(angle_diff);
        axis1.position_count += position_change;
        
        while (axis1.position_count < 0) {
            axis1.position_count += axis1.positions_per_rev;
        }
        axis1.position_count = axis1.position_count % axis1.positions_per_rev;
        
        axis1.updateQuadratureStates(position_change);
        
        if (axis1.position_count == 0) {
            axis1.index = 1;
        } else {
            axis1.index = 0;
        }
        
        if ((std::abs(current_angle_axis1) < ANGLE_TOLERANCE || std::abs(current_angle_axis1 - 360.0) < ANGLE_TOLERANCE) && axis1.channel_a == 0 && axis1.channel_b == 0) {
            axis1.index = 1;
            axis1.position_count = 0;
        }
        
        axis1.previous_angle = current_angle_axis1;
    }
    
    //Update axis 2 only if num_axes == 2
    if (num_axes == 2) {
        double angle_diff = current_angle_axis2 - axis2.previous_angle;
        while (angle_diff > 180.0) angle_diff -= 360.0;
        while (angle_diff < -180.0) angle_diff += 360.0;
        
        int position_change = axis2.angleToPositionChange(angle_diff);
        axis2.position_count += position_change;
        
        while (axis2.position_count < 0) {
            axis2.position_count += axis2.positions_per_rev;
        }
        axis2.position_count = axis2.position_count % axis2.positions_per_rev;
        
        axis2.updateQuadratureStates(position_change);
        
        if (axis2.position_count == 0) {
            axis2.index = 1;
        } else {
            axis2.index = 0;
        }
        
        if ((std::abs(current_angle_axis2) < ANGLE_TOLERANCE || std::abs(current_angle_axis2 - 360.0) < ANGLE_TOLERANCE) && axis2.channel_a == 0 && axis2.channel_b == 0) {
            axis2.index = 1;
            axis2.position_count = 0;
        }
        
        axis2.previous_angle = current_angle_axis2;
    }
}

//Get channel A for specified axis
int QuadratureOutput::getChannelA(int axis) const {
    if (axis == 0) return axis1.channel_a;
    if (axis == 1) return axis2.channel_a;
    return 0;
}

//Get channel B for specified axis
int QuadratureOutput::getChannelB(int axis) const {
    if (axis == 0) return axis1.channel_b;
    if (axis == 1) return axis2.channel_b;
    return 0;
}

//Get index for specified axis
int QuadratureOutput::getIndex(int axis) const {
    if (axis == 0) return axis1.index;
    if (axis == 1) return axis2.index;
    return 0;
}

//Set CPR for both axes
void QuadratureOutput::setCPR(int cpr) {
    int validated_cpr = std::max(MIN_CPR, std::min(cpr, MAX_CPR));
    axis1.cpr = validated_cpr;
    axis1.positions_per_rev = 4 * validated_cpr;
    axis1.position_count = 0;
    
    axis2.cpr = validated_cpr;
    axis2.positions_per_rev = 4 * validated_cpr;
    axis2.position_count = 0;
}

//Set CPR for a specific axis
void QuadratureOutput::setCPRAxis(int axis, int cpr) {
    int validated_cpr = std::max(MIN_CPR, std::min(cpr, MAX_CPR));
    if (axis == 0) {
        axis1.cpr = validated_cpr;
        axis1.positions_per_rev = 4 * validated_cpr;
        axis1.position_count = 0;
    } else if (axis == 1) {
        axis2.cpr = validated_cpr;
        axis2.positions_per_rev = 4 * validated_cpr;
        axis2.position_count = 0;
    }
}

//Get CPR for specified axis
int QuadratureOutput::getCPR(int axis) const {
    if (axis == 0) return axis1.cpr;
    if (axis == 1) return axis2.cpr;
    return 0;
}

//Get position count for specified axis
int QuadratureOutput::getPositionCount(int axis) const {
    if (axis == 0) return axis1.position_count;
    if (axis == 1) return axis2.position_count;
    return 0;
}

//Check if both axes are calibrated
bool QuadratureOutput::isCalibrated() const {
    return axis1.calibrated && axis2.calibrated;
}

//Check if specific axis is calibrated
bool QuadratureOutput::isCalibrated(int axis) const {
    if (axis == 0) return axis1.calibrated;
    if (axis == 1) return axis2.calibrated;
    return false;
}

//Reset index for specified axis
void QuadratureOutput::resetIndex(int axis, double current_angle) {
    if (axis == 0) {
        if ((std::abs(current_angle) < ANGLE_TOLERANCE || std::abs(current_angle - 360.0) < ANGLE_TOLERANCE) && axis1.channel_a == 0 && axis1.channel_b == 0) {
            axis1.index = 1;
            axis1.position_count = 0;
        }
    } else if (axis == 1 && num_axes == 2) {
        if ((std::abs(current_angle) < ANGLE_TOLERANCE || std::abs(current_angle - 360.0) < ANGLE_TOLERANCE) && axis2.channel_a == 0 && axis2.channel_b == 0) {
            axis2.index = 1;
            axis2.position_count = 0;
        }
    }
}

//Set number of axes (1 or 2)
void QuadratureOutput::setNumAxes(int num_axes_val) {
    num_axes = std::max(1, std::min(num_axes_val, 2));
}

//Get number of axes
int QuadratureOutput::getNumAxes() const {
    return num_axes;
}

//Get formatted output
QuadratureOutputFormat QuadratureOutput::getFormattedOutput() const {
    QuadratureOutputFormat output;
    output.axis1_A = axis1.channel_a;
    output.axis1_B = axis1.channel_b;
    output.axis2_A = (num_axes == 2) ? axis2.channel_a : 0;
    output.axis2_B = (num_axes == 2) ? axis2.channel_b : 0;
    output.index = axis1.index | (num_axes == 2 ? axis2.index : 0);
    return output;
}

//Get formatted output as CSV string
std::string QuadratureOutput::getFormattedOutputString() const {
    std::stringstream ss;
    if (num_axes == 1) {
        // single axis: A,B,Index
        ss << axis1.channel_a << ","
           << axis1.channel_b << ","
           << axis1.index;
    } else {
        // dual axis: A1,B1,A2,B2,Index
        ss << axis1.channel_a << ","
           << axis1.channel_b << ","
           << axis2.channel_a << ","
           << axis2.channel_b << ","
           << (axis1.index | axis2.index);
    }
    return ss.str();
}

//Validate axis index
bool QuadratureOutput::isValidAxis(int axis) const {
    return (axis == 0 || axis == 1);
}