// quadrature_decoder.c
// Authors: Alex White, Richard Harrison
// Date: 2026-03-24
// Version: 2
//
// This file contains the functionality of the quadrature decoding for the S26-14 project.
// Intended functionality: The quadrature output is the chosen method of outputting angle data from the sensors on the device. This method
// of output is not easily read by humans, and so a decoder is needed to take the quadrature output and convert it into a human-readable angle value.
// The angle range is 0 to 360. Mostly intended for use in testing the quadrature encoded output.

#include "quadrature_decoder.h"

#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const int VALID_TRANSITIONS[QUADRATURE_STATES][QUADRATURE_STATES] = {
    { 0,  1, -1,  0}, // from state 0: can go to 1 (fwd) or 2 (rev)
    {-1,  0,  0,  1}, // from state 1: can go to 3 (fwd) or 0 (rev)
    { 1,  0,  0, -1}, // from state 2: can go to 0 (fwd) or 3 (rev)
    { 0, -1,  1,  0}  // from state 3: can go to 2 (fwd) or 1 (rev)
};

static int EncodeState(int a, int b);
static int ClampCPR(int cpr);
static int GetDirection(int from, int to);

//static bool IsWithinAxis(int axis);

//============================================================================================
//                          Helper Implementation
//============================================================================================

static int ClampCPR(int cpr) {
    return MAX(MIN_CPR, MIN(cpr, MAX_CPR));
}

static int EncodeState(int a, int b) {
    return (a << 1) | b;
}

static int GetDirection(int from, int to) {
    if ((from < 0) || (from >= QUADRATURE_STATES) || (to < 0) || (to >= QUADRATURE_STATES)) return 0;
    return VALID_TRANSITIONS[from][to];
}

//============================================================================================
//                          QDecoderAxisState Implementation
//============================================================================================

QDecoderAxisState QDecoderAxisState_ConstructDefault() {
    return QDecoderAxisState_Construct(4096);
}

QDecoderAxisState QDecoderAxisState_Construct(int cpr) {
    QDecoderAxisState axis;

    axis.cpr = ClampCPR(cpr);
	axis.ppr = axis.cpr * 4LL;
    axis.count = 0;
    axis.absolute_count = 0;

    axis.last_channel_a = 0;
    axis.last_channel_b = 0;
    axis.last_index = 0;

    axis.synchronized = false;

    axis.pulse_count = 0;
    axis.index_pulse_count = 0;
    axis.error_count = 0;

    return axis;
}

void QDecoderAxisState_InitializeAtIndex(QDecoderAxisState* axis) {
    // if (axis < 0 || axis >= num_axes_) return;
    axis->absolute_count = 0;
    axis->synchronized = true;

    axis->last_channel_a = 0;
    axis->last_channel_b = 0;
    axis->last_index = 1;
}

void QDecoderAxisState_Reset(QDecoderAxisState* axis) {
    axis->count = 0;
    axis->absolute_count = 0;
    axis->synchronized = false;

    axis->last_channel_a = 0;
    axis->last_channel_b = 0;
    axis->last_index = 0;

    axis->pulse_count = 0;
    axis->error_count = 0;
    axis->index_pulse_count = 0;
}

void QDecoderAxisState_ProcessAxisPulse(QDecoderAxisState* axis, int a, int b, int index) {
    axis->pulse_count++;

    // Check for index pulse (transition from 0 to 1)
    if (index == 1 && axis->last_index == 0) {
        axis->index_pulse_count++;
        axis->synchronized = true;
        // Optionally reset position at index
        // axis->absolute_count = 0;
    }

    // Only process quadrature if state actually changed
    if (a == axis->last_channel_a && b == axis->last_channel_b) {
        return;  // No change, skip
    }

    // Check to make sure both channels are not changing at once
    if (a != axis->last_channel_a && b != axis->last_channel_b) {
        axis->error_count++; // only one bit can change at a time
        return;
    }

    int from_state = EncodeState(axis->last_channel_a, axis->last_channel_b);
    int to_state = EncodeState(a, b);

    // Validate transition
    int direction = GetDirection(from_state, to_state);

    if (direction == 0) { // Invalid transition detected
        axis->error_count++;
        // Don't update position on invalid transition
        return;
    }
    else if (direction > 0) { // Forward direction
        // i = (i + 1) % n; ~ Circular Increment
        axis->count = (axis->count + 1) % axis->ppr;
        axis->absolute_count++;
    }
    else { // Reverse direction
		// i = (i - 1 + n) % n; ~ Circular Decrement
        axis->count = (axis->count - 1 + axis->ppr) % axis->ppr;
        axis->absolute_count--;
    }

    axis->last_channel_a = a;
    axis->last_channel_b = b;
    axis->last_index = index;
}

bool QDecoderAxisState_HasErrors(QDecoderAxisState* axis) {
    return (axis->error_count > 0);
}

void QDecoderAxisState_ClearErrors(QDecoderAxisState* axis) {
    axis->error_count = 0;
}

long long QDecoderAxisState_GetPositionCount(const QDecoderAxisState* axis) {
    return axis->count;
}

long long QDecoderAxisState_GetRevolutionCount(const QDecoderAxisState* axis) {
    return (long long)(axis->absolute_count / axis->ppr);
}

double QDecoderAxisState_GetAngleDeg(const QDecoderAxisState* axis) {
    return ((double)(axis->count) / axis->ppr) * DEGREES_PER_REVOLUTION;
}

double QDecoderAxisState_GetAngleRad(const QDecoderAxisState* axis) {
    return ((double)(axis->count) / axis->ppr) * 2.0 * M_PI;
}

//============================================================================================
//                          QuadratureDecoder Implementation
//============================================================================================

QuadratureDecoder QuadratureDecoder_ConstructDefault() {
    return QuadratureDecoder_Construct(4096, 1);
}

QuadratureDecoder QuadratureDecoder_Construct(int cpr, int num_axes) {
    QuadratureDecoder decoder;
    decoder.num_axes = MIN(num_axes, MAX_AXES);

    cpr = ClampCPR(cpr); // Make sure to clamp

    int i;
    for (i = 0; i < MAX_AXES; i++) {
        decoder.axes[i] = QDecoderAxisState_Construct(cpr);
    }

    return decoder;
}

void QuadratureDecoder_SetCPR(QuadratureDecoder* decoder, int cpr) {
    cpr = ClampCPR(cpr);
    for (int i = 0; i < decoder->num_axes; i++) {
        decoder->axes[i].cpr = cpr;
		decoder->axes[i].ppr = cpr * 4LL;
    }
}

void QuadratureDecoder_SetNumAxes(QuadratureDecoder* decoder, int num_axes) {
    decoder->num_axes = MIN(MAX(num_axes, 1), MAX_AXES);
}

void QuadratureDecoder_ProcessPulse(QuadratureDecoder* decoder, int axis, int ch_a, int ch_b, int index) {
    if (axis < 0 || axis >= decoder->num_axes) return;
    QDecoderAxisState_ProcessAxisPulse(&decoder->axes[axis], ch_a, ch_b, index);
}

void QuadratureDecoder_ProcessPulseOutput(QuadratureDecoder* decoder, const QuadratureOutput* output) {
    QDecoderAxisState_ProcessAxisPulse(&decoder->axes[0], output->axis1.channel_a, output->axis1.channel_b, output->axis1.index);
    if (decoder->num_axes == MAX_AXES) {
        QDecoderAxisState_ProcessAxisPulse(&decoder->axes[1], output->axis2.channel_a, output->axis2.channel_b, output->axis2.index);
    }
}

void QuadratureDecoder_ProcessPulseChannels(QuadratureDecoder* decoder, int ch_a1, int ch_b1, int ch_a2, int ch_b2, int index) {
    QDecoderAxisState_ProcessAxisPulse(&decoder->axes[0], ch_a1, ch_b1, index);
    if (decoder->num_axes == MAX_AXES) {
        QDecoderAxisState_ProcessAxisPulse(&decoder->axes[1], ch_a2, ch_b2, index);
    }
}

void QuadratureDecoder_GetFormattedOutput(const QuadratureDecoder* decoder, int axis, char* buffer, size_t buffer_size) {
    if (axis < 0 || axis >= decoder->num_axes) {
        if (buffer_size > 0) {
            buffer[0] = '\0';
        }
        return;
    }

//    snprintf(buffer, buffer_size, "%.3f,%lld,%lld",
//             QDecoderAxisState_GetAngleDeg(&decoder->axes[axis]),
//             QDecoderAxisState_GetPositionCount(&decoder->axes[axis]),
//             QDecoderAxisState_GetRevolutionCount(&decoder->axes[axis])
//	);
}

void QuadratureDecoder_GetFormattedOutputDual(const QuadratureDecoder* decoder, char* buffer, size_t buffer_size) {
    if (decoder->num_axes <= 0) {
        if (buffer_size > 0) {
            buffer[0] = '\0';
        }
        return;
    }

    if (decoder->num_axes == 1) {
//        snprintf(buffer, buffer_size, "%.3f,%lld,%lld",
//                 QDecoderAxisState_GetAngleDeg(&decoder->axes[0]),
//                 QDecoderAxisState_GetPositionCount(&decoder->axes[0]),
//                 QDecoderAxisState_GetRevolutionCount(&decoder->axes[0]));
    }
    else {
//        snprintf(buffer, buffer_size, "%.3f,%lld,%lld,%.3f,%lld,%lld",
//                 QDecoderAxisState_GetAngleDeg(&decoder->axes[0]),
//                 QDecoderAxisState_GetPositionCount(&decoder->axes[0]),
//                 QDecoderAxisState_GetRevolutionCount(&decoder->axes[0]),
//                 QDecoderAxisState_GetAngleDeg(&decoder->axes[1]),
//                 QDecoderAxisState_GetPositionCount(&decoder->axes[1]),
//                 QDecoderAxisState_GetRevolutionCount(&decoder->axes[1]));
    }
}
