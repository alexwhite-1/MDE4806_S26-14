// Author: Alex White, 2/2/2026
//         Richard Harrison, 03/12/2026
// Version: 2
// This file is the header file for the quadrature output functionality. See other file for description.
#ifndef QUAD_OUTPUT_H
#define QUAD_OUTPUT_H

#include <stdbool.h>
#include <string.h>

// Structure to hold formatted quadrature output: Axis 1 A, Axis 1 B, Axis 2 A, Axis 2 B, Index
typedef struct {
    int axis1_A;   // Axis 1 Channel A
    int axis1_B;   // Axis 1 Channel B
    int axis2_A;   // Axis 2 Channel A
    int axis2_B;   // Axis 2 Channel B
    int index;     // Index signal (shared or primary)
} QuadratureOutputFormat;

// Internal structure for single-axis state
typedef struct {
    int cpr;
    int positions_per_rev;
    double starting_angle;
    double previous_angle;
    int position_count;
    int channel_a;
    int channel_b;
    int index;
    bool calibrated;
} AxisState;

AxisState AxisState_Construct(int cpr_val = 4096);
void AxisState_UpdateAxis(AxisState* axis, double current_angle_axis);
int AxisState_AngleToPositionChange(AxisState* state, double angle_diff);
void AxisState_UpdateQuadratureStates(AxisState* state, int position_change);
void AxisState_GetQuadraturePattern(AxisState* state);
void AxisState_ResetIndex(AxisState* axis, double angle);
void AxisState_SetCPR(AxisState* axis, int cpr_val);

typedef struct {
    AxisState axis1;  // First axis (e.g. X)
    AxisState axis2;  // Second axis (e.g. Y)
    int num_axes;     // Number of axes to operate: 1 or 2 (default 2)
} QuadratureOutput;

// Constructor
QuadratureOutput QuadratureOutput_Construct(int cpr = 4096, int num_axes_init = 2);

// Initialize both axes with starting angles
void QuadratureOutput_Initialize(QuadratureOutput* output, double initial_angle_axis1, double initial_angle_axis2);

// Update both axes with new angle readings
void QuadratureOutput_Update(QuadratureOutput* output, double current_angle_axis1, double current_angle_axis2);

// Set number of axes
void QuadratureOutput_SetNumAxes(QuadratureOutput* output, int num_axes);

// Get formatted output as: Axis 1 A, Axis 1 B, Axis 2 A, Axis 2 B, Index
QuadratureOutputFormat QuadratureOutput_GetFormattedOutput(const QuadratureOutput* output);

// Get formatted output as CSV string (adapts to 1 or 2 axes)
void QuadratureOutput_GetFormattedOutputString(const QuadratureOutput* output);

#endif 