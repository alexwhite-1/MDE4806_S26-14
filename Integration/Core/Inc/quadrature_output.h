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

// Queue size for holding quadrature states (65536)
#define QUADRATURE_STATE_QUEUE_SIZE 65536

// Quadrature Gray Code Sequence (standard quadrature encoding):
// Position 0: A=0, B=0 (binary 00)
// Position 1: A=1, B=0 (binary 10) - A leads for forward rotation
// Position 2: A=1, B=1 (binary 11)
// Position 3: A=0, B=1 (binary 01) - B leads for backward rotation
//
// Forward Rotation (positive angle change):
//   Channel A leads Channel B by 90 degrees
//   Sequence: 00 → 10 → 11 → 01 → 00 → ...
//
// Backward Rotation (negative angle change):
//   Channel B leads Channel A by 90 degrees
//   Sequence: 00 → 01 → 11 → 10 → 00 → ...
//
// The queue stores only Axis 1 A and B channels to ensure all state changes
// are captured in the correct order for reliable angle reconstruction.

// Structure to hold a single quadrature state (Axis 1 A and B only)
typedef struct {
    int axis1_A;   // Axis 1 Channel A
    int axis1_B;   // Axis 1 Channel B
} QuadratureState;

// Queue structure for holding quadrature states
typedef struct {
    QuadratureState states[QUADRATURE_STATE_QUEUE_SIZE];
    size_t head;   // Points to the next write position
    size_t tail;   // Points to the next read position
    size_t count;  // Number of elements currently in the queue
} QuadratureStateQueue;

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
    QuadratureStateQueue queue;  // Queue for holding quadrature states
} QuadratureOutput;

//============================================================================================
//                          QuadratureStateQueue Implementation
//============================================================================================

// Initialize queue
void QuadratureStateQueue_Initialize(QuadratureStateQueue* queue);

// Enqueue a state
bool QuadratureStateQueue_Enqueue(QuadratureStateQueue* queue, QuadratureState state);

// Dequeue a state
bool QuadratureStateQueue_Dequeue(QuadratureStateQueue* queue, QuadratureState* state);

// Check if queue is empty
bool QuadratureStateQueue_IsEmpty(const QuadratureStateQueue* queue);

// Verify that quadrature state is a valid Gray code state
// Returns true if the state is one of: (0,0), (1,0), (1,1), (0,1)
// Returns false for invalid states like (1,1,0) or (0,0,1)
bool QuadratureState_IsValid(QuadratureState state);

// Verify that a state transition is valid in Gray code
// Valid transitions: only one bit should change between consecutive states
// Returns true if transition is valid, false if more than one bit changed
bool QuadratureState_IsValidTransition(QuadratureState from, QuadratureState to);

// Verify that the queue contains valid Gray code sequence
// This is a diagnostic function for testing and validation
void QuadratureStateQueue_VerifySequence(const QuadratureStateQueue* queue);

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

// Update axis with new angle reading and enqueue states
void QOutputAxisState_UpdateAxisWithQueue(QOutputAxisState* axis, double angle_axis, QuadratureStateQueue* queue);

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

// Dequeue a single state from the output queue
bool QuadratureOutput_DequeueState(QuadratureOutput* output, QuadratureState* state);

// Get the number of pending states in the queue
size_t QuadratureOutput_GetQueueSize(const QuadratureOutput* output);

// Check if the queue is empty
bool QuadratureOutput_IsQueueEmpty(const QuadratureOutput* output);

// Check if the queue is full
bool QuadratureOutput_IsQueueFull(const QuadratureOutput* output);

// Clear all pending states in the queue
void QuadratureOutput_ClearQueue(QuadratureOutput* output);

//step once in the direction of dir (1 for forward, -1 for backward)
void QOutputAxisState_StepOne(QOutputAxisState* axis, int dir);


#ifdef __cplusplus
}
#endif

#endif 
