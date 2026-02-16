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

//Structure to hold formatted quadrature output: Axis 1 A, Axis 1 B, Axis 2 A, Axis 2 B, Index
struct QuadratureOutputFormat {
    int axis1_A;   //Axis 1 Channel A
    int axis1_B;   //Axis 1 Channel B
    int axis2_A;   //Axis 2 Channel A
    int axis2_B;   //Axis 2 Channel B
    int index;     //Index signal (shared or primary)
};

//Class to manage quadrature output for dual axes (e.g., X and Y or two joints)
class QuadratureOutput {
private:
    //Internal structure for single-axis state
    struct AxisState {
        int cpr;
        int positions_per_rev;
        double starting_angle;
        double previous_angle;
        int position_count;
        int channel_a;
        int channel_b;
        int index;
        bool calibrated;
        
        //Constructor
        AxisState(int cpr_val = 4096);
        
        //Helper methods
        int angleToPositionChange(double angle_diff);
        void updateQuadratureStates(int position_change);
        void getQuadraturePattern(int position, int& a, int& b) const;
    };
    
    AxisState axis1;  //first axis (e.g. X)
    AxisState axis2;  //second axis (e.g. Y)
    int num_axes;     //number of axes to operate: 1 or 2 (default 2)
    
public:
    //Constructor
    QuadratureOutput(int cpr = 4096, int num_axes_init = 2);
    
    //Initialize both axes with starting angles
    void initialize(double initial_angle_axis1, double initial_angle_axis2);
    
    //Update both axes with new angle readings
    void update(double current_angle_axis1, double current_angle_axis2);
    
    //Get channel A for specified axis (0 or 1)
    int getChannelA(int axis) const;
    
    //Get channel B for specified axis (0 or 1)
    int getChannelB(int axis) const;
    
    //Get index signal for specified axis (0 or 1)
    int getIndex(int axis) const;
    
    //Set CPR for both axes
    void setCPR(int cpr);
    
    //Set CPR for a specific axis
    void setCPRAxis(int axis, int cpr);
    
    //Get CPR for specified axis
    int getCPR(int axis) const;
    
    //Get position count for specified axis
    int getPositionCount(int axis) const;
    
    //Check if both axes are calibrated
    bool isCalibrated() const;
    
    //Check if specific axis is calibrated
    bool isCalibrated(int axis) const;
    
    //Reset index for specified axis
    void resetIndex(int axis, double current_angle);
    
    //Set number of axes (1 or 2)
    void setNumAxes(int num_axes_val);
    
    //Get number of axes
    int getNumAxes() const;
    
    //Get formatted output as: Axis 1 A, Axis 1 B, Axis 2 A, Axis 2 B, Index
    QuadratureOutputFormat getFormattedOutput() const;
    
    //Get formatted output as CSV string (adapts to 1 or 2 axes)
    std::string getFormattedOutputString() const;
    
private:
    //Helper to validate axis index
    bool isValidAxis(int axis) const;
};

#endif