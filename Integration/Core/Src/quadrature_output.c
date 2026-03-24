// quadrature_output.c
// Authors: Alex White, Richard Harrison
// Date: 2026-03-24
// Version: 2
//
// This file contains the functionality of the quadrature encoded output for the S26-14 project.
// Intended functionality: The quadrature output will serve as the method of relying the positional data from the sensors on the PCB
// to the robot via two output pins. With positive rotation, channel A will lead channel B by 90 degrees and vice versa for negative
// rotation. The quadrature resolution may be set to any value from 1 to 9000 CPR (cycles per revolution)
// corresponding to 4 to 36000 positions per revolution. When the resolution is set to 9000 CPR, one quadrature state change on the
// A/B outputs corresponds to 0.01 degree. The PCB calculates the absolute position at a 640 Hz rate and updates the A/B signals
// in bursts of pulses at an average rate of approximately 6900 quadrature states per second. It is important to note that the
// quadrature signal itself does not necessarily indicate what the absolute angle is, simply the change in angle from a previous
// value. Therefore, it is very important that the calibration procedure functions properly and the inclinometer knows the angle it
// is starting from. The quadrature functionality does not know or care how the angle is calculated, simply that it is provided with
// an angle. Intended to be used in conjunction with the quadrature decoder function for proper testing and verification.

#include "quadrature_output.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "quadrature_common.h"

//============================================================================================
//                          AxisState Implementation
//============================================================================================

// AxisState Constructor
QOutputAxisState QOutputAxisState_ConstructDefault() {
    return QOutputAxisState_Construct(4096);
}

QOutputAxisState QOutputAxisState_Construct(int cpr_val) {
    QOutputAxisState state;
    state.cpr = MAX(MIN_CPR, MIN(cpr_val, MAX_CPR));

    state.positions_per_rev = state.cpr * 4;

    state.starting_angle = 0.0;
    state.previous_angle = 0.0;
    state.position_count = 0;

    state.channel_a = 0;
    state.channel_b = 0,
    state.index = 0,

    state.calibrated = false;
    return state;
}

// Initialize axis with starting angle
void QOutputAxisState_Initialize(QOutputAxisState* axis, double angle) {
    // Initialize axis
    axis->starting_angle = angle;
    axis->previous_angle = angle;
    axis->position_count = (angle / 360) * axis->positions_per_rev;

    // Assuming the angle is calibrated upon the beginning of the quadrature
    axis->calibrated = true;

    // Starting low, as there has been no change yet
    axis->channel_a = 0;
    axis->channel_b = 0;

    // Since the channels are low, index can start high IF AND ONLY IF the starting angle is 0/360
    axis->index = (axis->starting_angle == 0 || axis->starting_angle == 360) ? 1 : 0;
}

// Update axes with new angle readings
void QOutputAxisState_UpdateAxis(QOutputAxisState* axis, double angle_axis) {
    double angle_diff = angle_axis - axis->previous_angle;
    while (angle_diff > 180.0) angle_diff -= 360.0;
    while (angle_diff < -180.0) angle_diff += 360.0;

    int position_change = QOutputAxisState_AngleToPositionChange(axis, angle_diff);
    axis->position_count += position_change;

    while (axis->position_count < 0) {
        axis->position_count += axis->positions_per_rev;
    }
    axis->position_count = axis->position_count % axis->positions_per_rev;

    QOutputAxisState_UpdateQuadratureStates(axis, position_change);

    axis->index = (axis->position_count == 0 || axis->position_count == (axis->positions_per_rev - 1)) ? 1 : 0;

    QOutputAxisState_ResetIndex(axis, angle_axis);

    axis->previous_angle = angle_axis;
}

// Convert angle difference to position change
int QOutputAxisState_AngleToPositionChange(QOutputAxisState* state, double angle_diff) {
    double raw_position_change = (angle_diff / DEGREES_PER_REVOLUTION) * state->positions_per_rev;
    return (int)round(raw_position_change);
}

// Update quadrature states based on position change
void QOutputAxisState_UpdateQuadratureStates(QOutputAxisState* state, int position_change) {
    if (position_change == 0) return;
    QOutputAxisState_GetQuadraturePattern(state);
}

// Get the quadrature pattern for a position
void QOutputAxisState_GetQuadraturePattern(QOutputAxisState* state)  {
    switch (state->position_count % 4)
    {
    case 0:
        state->channel_a = 0;
        state->channel_b = 0;
        break;
    case 1:
        state->channel_a = 1;
        state->channel_b = 0;
        break;
    case 2:
        state->channel_a = 1;
        state->channel_b = 1;
        break;
    case 3:
        state->channel_a = 0;
        state->channel_b = 1;
        break;
    default:
        state->channel_a = 0;
        state->channel_b = 0;
        break;
    }
}

// Reset index based on angle
void QOutputAxisState_ResetIndex(QOutputAxisState* axis, double angle) {
    if ((fabs(angle) < ANGLE_TOLERANCE || fabs(angle - 360.0) < ANGLE_TOLERANCE) && axis->channel_a == 0 && axis->channel_b == 0) {
        axis->index = 1;
        axis->position_count = 0;
    }
}

// Set CPR
void QOutputAxisState_SetCPR(QOutputAxisState* axis, int cpr) {
    int validated_cpr = MAX(MIN_CPR, MIN(cpr, MAX_CPR));
    axis->cpr = validated_cpr;
    axis->positions_per_rev = 4 * validated_cpr;
    axis->position_count = 0;
}

//============================================================================================
//                          QuadratureOutput Implementation
//============================================================================================

// Validate axis index
bool QuadratureOutput_IsValidAxis(int axis) {
    return (axis == 0 || axis == 1);
}

// Constructor
QuadratureOutput QuadratureOutput_ConstructDefault() {
    return QuadratureOutput_Construct(4096, 2);
}

QuadratureOutput QuadratureOutput_ConstructCPR(int cpr) {
    return QuadratureOutput_Construct(cpr, 2);
}

QuadratureOutput QuadratureOutput_Construct(int cpr, int num_axes) {
    QuadratureOutput output;

    output.axis1 = QOutputAxisState_Construct(cpr);
    output.axis2 = QOutputAxisState_Construct(cpr);
    output.num_axes = MAX(1, MIN(num_axes, 2));

    return output;
}

// Initialize both axes with starting angles
void QuadratureOutput_Initialize(QuadratureOutput* output, double angle_axis1, double angle_axis2) {
    // Initialize axis 1 (always)
    QOutputAxisState_Initialize(&output->axis1, angle_axis1);

    // Initialize axis 2 (only if num_axes == 2)
    if (output->num_axes == 2) QOutputAxisState_Initialize(&output->axis2, angle_axis2);
}

// Update both axes with new angle readings
void QuadratureOutput_Update(QuadratureOutput* output, double angle_axis1, double angle_axis2) {
    // Check for calibration, exit early if not calibrated
    if (!output->axis1.calibrated || (output->num_axes == 2 && !output->axis2.calibrated)) {
        printf("One or more axes are not calibrated. Please calibrate and try again.\n");
        return;
    }

    // Update axis 1 (always)
    QOutputAxisState_UpdateAxis(&output->axis1, angle_axis1);

    // Update axis 2 (only if num_axes == 2)
    if (output->num_axes == 2) QOutputAxisState_UpdateAxis(&output->axis2, angle_axis2);
}

// Get formatted output as: Axis 1 A, Axis 1 B, Axis 2 A, Axis 2 B, Index
QuadratureOutputFormat QuadratureOutput_GetFormattedOutput(const QuadratureOutput* output) {
    QuadratureOutputFormat outputFormatted;

    outputFormatted.axis1_A = output->axis1.channel_a;
    outputFormatted.axis1_B = output->axis1.channel_b;
    outputFormatted.axis2_A = (output->num_axes == 2) ? output->axis2.channel_a : 0;
    outputFormatted.axis2_B = (output->num_axes == 2) ? output->axis2.channel_b : 0;
    outputFormatted.index = output->axis1.index | (output->num_axes == 2 ? output->axis2.index : 0);

    return outputFormatted;
}

// Get formatted output as CSV string (adapts to 1 or 2 axes)
void QuadratureOutput_GetFormattedOutputString(const QuadratureOutput* output, char* buffer, size_t buffer_size) {
    if (output->num_axes == 1) {
        snprintf(buffer, buffer_size, "%d,%d,%d",
                 output->axis1.channel_a,
                 output->axis1.channel_b,
                 output->axis1.index);
    }
    else {
        snprintf(buffer, buffer_size, "%d,%d,%d,%d,%d",
                 output->axis1.channel_a,
                 output->axis1.channel_b,
                 output->axis2.channel_a,
                 output->axis2.channel_b,
                 (output->axis1.index | output->axis2.index));
    }
}

// Set number of axes
void QuadratureOutput_SetNumAxes(QuadratureOutput* output, int num_axes) {
    output->num_axes = MAX(1, MIN(num_axes, 2));
}