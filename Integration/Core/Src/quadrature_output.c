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
//                          QuadratureStateQueue Implementation
//============================================================================================

// Initialize queue
void QuadratureStateQueue_Initialize(QuadratureStateQueue* queue) {
    if (queue == NULL) return;
    
    memset(queue->states, 0, sizeof(queue->states));
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

// Enqueue a state
bool QuadratureStateQueue_Enqueue(QuadratureStateQueue* queue, QuadratureState state) {
    if (queue == NULL) return false;
    
    // If queue is full, return false (do not overwrite)
    if (queue->count >= QUADRATURE_STATE_QUEUE_SIZE) {
        return false;
    }
    
    // Add state to queue
    queue->states[queue->head] = state;
    queue->head = (queue->head + 1) % QUADRATURE_STATE_QUEUE_SIZE;
    queue->count++;
    
    return true;
}

// Dequeue a state
bool QuadratureStateQueue_Dequeue(QuadratureStateQueue* queue, QuadratureState* state) {
    if (queue == NULL || state == NULL) return false;
    
    // If queue is empty, return false
    if (queue->count == 0) {
        return false;
    }
    
    // Get state from queue
    *state = queue->states[queue->tail];
    queue->tail = (queue->tail + 1) % QUADRATURE_STATE_QUEUE_SIZE;
    queue->count--;
    
    return true;
}

// Check if queue is empty
bool QuadratureStateQueue_IsEmpty(const QuadratureStateQueue* queue) {
    if (queue == NULL) return true;
    return queue->count == 0;
}

// Check if queue is full
bool QuadratureStateQueue_IsFull(const QuadratureStateQueue* queue) {
    if (queue == NULL) return false;
    return queue->count >= QUADRATURE_STATE_QUEUE_SIZE;
}

// Get current queue size
size_t QuadratureStateQueue_GetSize(const QuadratureStateQueue* queue) {
    if (queue == NULL) return 0;
    return queue->count;
}

// Clear the queue
void QuadratureStateQueue_Clear(QuadratureStateQueue* queue) {
    if (queue == NULL) return;
    
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

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

    int steps = QOutputAxisState_AngleToPositionChange(axis, angle_diff);
    int dir = (steps >= 0) ? 1 : -1;
    int remaining = (steps >= 0) ? steps : -steps;

    while (remaining--)
        QOutputAxisState_StepOne(axis, dir);

    axis->previous_angle = angle_axis;
}

// Update axis with new angle reading and enqueue states
// This function ensures all intermediate quadrature states are captured in order
// by stepping through one position at a time and enqueueing each resulting state.
// This guarantees correctness in state reconstruction even with rapid angle changes.
void QOutputAxisState_UpdateAxisWithQueue(QOutputAxisState* axis, double angle_axis, QuadratureStateQueue* queue) {
    double angle_diff = angle_axis - axis->previous_angle;
    while (angle_diff > 180.0) angle_diff -= 360.0;
    while (angle_diff < -180.0) angle_diff += 360.0;

    int steps = QOutputAxisState_AngleToPositionChange(axis, angle_diff);
    int dir = (steps >= 0) ? 1 : -1;
    int remaining = (steps >= 0) ? steps : -steps;

    // Step through each position change and capture the resulting state
    while (remaining--) {
        QOutputAxisState_StepOne(axis, dir);
        
        // Enqueue the state after each step (correctness priority)
        if (queue != NULL) {
            QuadratureState state;
            state.axis1_A = axis->channel_a;
            state.axis1_B = axis->channel_b;
            
            if (!QuadratureStateQueue_Enqueue(queue, state)) {
                // Queue is full - state not captured. This indicates the consumer
                // is not dequeuing fast enough. Consider increasing queue size.
                printf("Warning: Quadrature state queue is full, state not enqueued\n");
            }
        }
    }

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
// Gray code sequence for quadrature encoding:
//   Position 0: A=0, B=0 (state 00)
//   Position 1: A=1, B=0 (state 10) - A leads for forward rotation
//   Position 2: A=1, B=1 (state 11)
//   Position 3: A=0, B=1 (state 01) - B leads for backward rotation
// Forward rotation produces: 00 → 10 → 11 → 01 → 00 (A leads B by 90°)
// Backward rotation produces: 00 → 01 → 11 → 10 → 00 (B leads A by 90°)
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
    
    // Initialize the queue
    QuadratureStateQueue_Initialize(&output.queue);

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

    // Update axis 1 (always) and enqueue states
    QOutputAxisState_UpdateAxisWithQueue(&output->axis1, angle_axis1, &output->queue);

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

// Dequeue a single state from the output queue
bool QuadratureOutput_DequeueState(QuadratureOutput* output, QuadratureState* state) {
    if (output == NULL || state == NULL) return false;
    return QuadratureStateQueue_Dequeue(&output->queue, state);
}

// Get the number of pending states in the queue
size_t QuadratureOutput_GetQueueSize(const QuadratureOutput* output) {
    if (output == NULL) return 0;
    return QuadratureStateQueue_GetSize(&output->queue);
}

// Check if the queue is empty
bool QuadratureOutput_IsQueueEmpty(const QuadratureOutput* output) {
    if (output == NULL) return true;
    return QuadratureStateQueue_IsEmpty(&output->queue);
}

// Check if the queue is full
bool QuadratureOutput_IsQueueFull(const QuadratureOutput* output) {
    if (output == NULL) return false;
    return QuadratureStateQueue_IsFull(&output->queue);
}

// Clear all pending states in the queue
void QuadratureOutput_ClearQueue(QuadratureOutput* output) {
    if (output == NULL) return;
    QuadratureStateQueue_Clear(&output->queue);
}

// moves position_count by exactly one step, wraps it, refreshes A/B,
// and asserts the index pulse when the encoder returns to position 0.
void QOutputAxisState_StepOne(QOutputAxisState* axis, int dir) {
    axis->position_count += dir;

    if (axis->position_count < 0)
        axis->position_count += axis->positions_per_rev;
    else if (axis->position_count >= axis->positions_per_rev)
        axis->position_count -= axis->positions_per_rev;

    QOutputAxisState_GetQuadraturePattern(axis);

    axis->index = (axis->position_count == 0 &&
        axis->channel_a == 0 &&
        axis->channel_b == 0) ? 1 : 0;
}
