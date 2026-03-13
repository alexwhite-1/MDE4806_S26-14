#include "quad_output.h"

#include <math.h>
#include <stdio.h>

// Macros
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

// Constants
const double DEGREES_PER_REVOLUTION = 360.0;
const int MAX_CPR = 9000;
const int MIN_CPR = 1;
const double ANGLE_TOLERANCE = 0.001; // tolerance for floating point comparisons

//============================================================================================
//                          AxisState Implementation
//============================================================================================

// AxisState Constructor
AxisState AxisState_Construct(int cpr_val) {
    AxisState state;
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

// Update axes with new angle readings
void AxisState_UpdateAxis(AxisState* axis, double current_angle_axis) {
    double angle_diff = current_angle_axis - axis->previous_angle;
    while (angle_diff > 180.0) angle_diff -= 360.0;
    while (angle_diff < -180.0) angle_diff += 360.0;

    int position_change = AxisState_AngleToPositionChange(axis, angle_diff);
    axis->position_count += position_change;

    while (axis->position_count < 0) {
        axis->position_count += axis->positions_per_rev;
    }
    axis->position_count = axis->position_count % axis->positions_per_rev;

    AxisState_UpdateQuadratureStates(axis, position_change);

    axis->index = (axis->position_count == 0 || axis->position_count == (axis->positions_per_rev - 1)) ? 1 : 0;

    AxisState_ResetIndex(axis, current_angle_axis);

    axis->previous_angle = current_angle_axis;
}


// AxisState: Convert angle difference to position change
int AxisState_AngleToPositionChange(AxisState* state, double angle_diff) {
    double raw_position_change = (angle_diff / DEGREES_PER_REVOLUTION) * state->positions_per_rev;
    return (int)round(raw_position_change);
}

// AxisState: Update quadrature states based on position change
void AxisState_UpdateQuadratureStates(AxisState* state, int position_change) {
    if (position_change == 0) return;
    AxisState_GetQuadraturePattern(state);
}

// AxisState: Get the quadrature pattern for a position
void AxisState_GetQuadraturePattern(AxisState* state)  {
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

void AxisState_ResetIndex(AxisState* axis, double angle) {
    if ((fabs(angle) < ANGLE_TOLERANCE || fabs(angle - 360.0) < ANGLE_TOLERANCE) && axis->channel_a == 0 && axis->channel_b == 0) {
        axis->index = 1;
        axis->position_count = 0;
    }
}

void AxisState_SetCPR(AxisState* axis, int cpr) {
    int validated_cpr = MAX(MIN_CPR, MIN(cpr, MAX_CPR));
    axis->cpr = validated_cpr;
    axis->positions_per_rev = 4 * validated_cpr;
    axis->position_count = 0;
}

//============================================================================================
//                          QuadratureOutput Main Implementation
//============================================================================================

// Validate axis index
bool QuadratureOutput_IsValidAxis(int axis) {
    return (axis == 0 || axis == 1);
}

// Constructor
QuadratureOutput QuadratureOutput_Construct(int cpr, int num_axes_init) {
    QuadratureOutput output;

    output.axis1 = AxisState_Construct(cpr);
    output.axis2 = AxisState_Construct(cpr);
    output.num_axes = MAX(1, MIN(num_axes_init, 2));

    return output;
}

// Initialize both axes with starting angles
void QuadratureOutput_Initialize(QuadratureOutput* output, double initial_angle_axis1, double initial_angle_axis2) {
    // Initialize axis 1
    output->axis1.starting_angle = initial_angle_axis1;
    output->axis1.previous_angle = initial_angle_axis1;
    output->axis1.position_count = (initial_angle_axis1 / 360) * (QuadratureOutput_getCPR(output, 0) * 4);

    // Assuming the angle is calibrated upon the beginning of the quadrature
    output->axis1.calibrated = true;

    // Starting low, as there has been no change yet
    output->axis1.channel_a = 0;
    output->axis1.channel_b = 0;

    // Since the channels are low, index can start high IF AND ONLY IF the starting angle is 0/360
    if (output->axis1.starting_angle == 0 || output->axis1.starting_angle == 360)
        output->axis1.index = 1;
    else
        output->axis1.index = 0;

    // Initialize axis 2 only if num_axes == 2
    if (output->num_axes == 2)
    {
        output->axis2.starting_angle = initial_angle_axis2;
        output->axis2.previous_angle = initial_angle_axis2;
        output->axis2.position_count = (initial_angle_axis2 / 360) * (QuadratureOutput_getCPR(output, 1) * 4);

        // Assuming the angle is calibrated upon the beginning of the quadrature
        output->axis2.calibrated = true;
        
        // Starting low, as there has been no change yet
        output->axis2.channel_a = 0;
        output->axis2.channel_b = 0;

        // Since the channels are low, index can start high IF AND ONLY IF the starting angle is 0/360
        if (output->axis2.starting_angle == 0 || output->axis2.starting_angle == 360)
            output->axis2.index = 1;
        else
            output->axis2.index = 0;
    }
}

// Update both axes with new angle readings
void QuadratureOutput_Update(QuadratureOutput* output, double current_angle_axis1, double current_angle_axis2) {
    // Check for calibration, exit early if not calibrated
    if (!output->axis1.calibrated || (output->num_axes == 2 && !output->axis2.calibrated)) {
        printf("One or more axes are not calibrated. Please calibrate and try again.\n");
        return;
    }

    // Update axis 1 (always)
    AxisState_UpdateAxis(&output->axis1, current_angle_axis1);

    // Update axis 2 (only if num_axes == 2)
    if (output->num_axes == 2) AxisState_UpdateAxis(&output->axis2, current_angle_axis2);
}

// Set number of axes
void QuadratureOutput_SetNumAxes(QuadratureOutput* output, int num_axes) {
    output->num_axes = MAX(1, MIN(num_axes, 2));
}

QuadratureOutputFormat QuadratureOutput_GetFormattedOutput(const QuadratureOutput* output) {
    QuadratureOutputFormat outputFormatted;

    outputFormatted.axis1_A = output->axis1.channel_a;
    outputFormatted.axis1_B = output->axis1.channel_b;
    outputFormatted.axis2_A = (output->num_axes == 2) ? output->axis2.channel_a : 0;
    outputFormatted.axis2_B = (output->num_axes == 2) ? output->axis2.channel_b : 0;
    outputFormatted.index = output->axis1.index | (output->num_axes == 2 ? output->axis2.index : 0);

    return outputFormatted;
}

void QuadratureOutput_GetFormattedOutputString(const QuadratureOutput* output, char* buffer, size_t buffer_size) {
    if (output->num_axes == 1)
    {
        snprintf(buffer, buffer_size, "%d,%d,%d",
                 output->axis1.channel_a,
                 output->axis1.channel_b,
                 output->axis1.index);
    }
    else
    {
        snprintf(buffer, buffer_size, "%d,%d,%d,%d,%d",
                 output->axis1.channel_a,
                 output->axis1.channel_b,
                 output->axis2.channel_a,
                 output->axis2.channel_b,
                 (output->axis1.index | output->axis2.index));
    }
}