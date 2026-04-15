// quadrature_output.h
// Authors: Alex White, Richard Harrison
// Date: 2026-03-24
// Version: 2
//
// This file is the header file for the quadrature output functionality.
// It contains the public data structures, constants, and function prototypes
// used to generate quadrature-encoded A/B/index output signals from angle data.
// Intended for use with the quadrature output source file and for integration
// with external modules such as the quadrature decoder and test code.

#ifndef QUADRATURE_OUTPUT_H
#define QUADRATURE_OUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>

// Structure to hold formatted quadrature output: Axis 1 A, Axis 1 B, Axis 2 A, Axis 2 B, Index
typedef struct {
    int axis1_A;   // Axis 1 Channel A
    int axis1_B;   // Axis 1 Channel B
    int axis2_A;   // Axis 2 Channel A
    int axis2_B;   // Axis 2 Channel B
    int index;     // Index signal (shared or primary)
} QuadratureOutputFormat;

// Structure for single-axis state
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
} QOutputAxisState;

typedef struct {
    QOutputAxisState axis1;  // First axis (e.g. X)
    QOutputAxisState axis2;  // Second axis (e.g. Y)
    int num_axes;     // Number of axes to operate: 1 or 2 (default 2)
} QuadratureOutput;

//============================================================================================
//                          AxisState Implementation
//============================================================================================

// Constructor
QOutputAxisState QOutputAxisState_ConstructDefault();
QOutputAxisState QOutputAxisState_Construct(int cpr_val);

// Initialize axis with starting angle
void QOutputAxisState_Initialize(QOutputAxisState* axis, double angle);

// Update axis with new angle reading
void QOutputAxisState_UpdateAxis(QOutputAxisState* axis, double angle_axis);

// Convert angle difference to position change
int QOutputAxisState_AngleToPositionChange(QOutputAxisState* state, double angle_diff);

// Update quadrature states based on position change
void QOutputAxisState_UpdateQuadratureStates(QOutputAxisState* state, int position_change);

// Get quadrature pattern for current state
void QOutputAxisState_GetQuadraturePattern(QOutputAxisState* state);

// Reset index based on angle
void QOutputAxisState_ResetIndex(QOutputAxisState* axis, double angle);

// Set CPR
void QOutputAxisState_SetCPR(QOutputAxisState* axis, int cpr_val);

//============================================================================================
//                          QuadratureOutput Implementation
//============================================================================================

// Constructor
QuadratureOutput QuadratureOutput_ConstructDefault();
QuadratureOutput QuadratureOutput_ConstructCPR(int cpr);
QuadratureOutput QuadratureOutput_Construct(int cpr, int num_axes);

// Initialize both axes with starting angles
void QuadratureOutput_Initialize(QuadratureOutput* output, double angle_axis1, double angle_axis2);

// Update both axes with new angle readings
void QuadratureOutput_Update(QuadratureOutput* output, double angle_axis1, double angle_axis2);

// Get formatted output as: Axis 1 A, Axis 1 B, Axis 2 A, Axis 2 B, Index
QuadratureOutputFormat QuadratureOutput_GetFormattedOutput(const QuadratureOutput* output);

// Get formatted output as CSV string (adapts to 1 or 2 axes)
void QuadratureOutput_GetFormattedOutputString(const QuadratureOutput* output, char* buffer, size_t buffer_size);

// Set number of axes
void QuadratureOutput_SetNumAxes(QuadratureOutput* output, int num_axes);

//step once in the direction of dir (1 for forward, -1 for backward)
void QOutputAxisState_StepOne(QOutputAxisState* axis, int dir);


#ifdef __cplusplus
}
#endif

#endif
