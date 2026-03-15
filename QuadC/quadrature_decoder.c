#include "quadrature_decoder.h"

#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const int VALID_TRANSITIONS[4][4] = {
    {0, -1,  1, -1},
    {1,  0, -1,  1},
    {-1, 1,  0, -1},
    {-1,-1,  1,  0}
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
    // if (axis < 0 || axis >= num_axes_) return;
    axis->absolute_count = 0;
    axis->synchronized = false;

    axis->last_channel_a = 0;
    axis->last_channel_b = 0;
    axis->last_index = 0;

    axis->pulse_count = 0;
    axis->index_pulse_count = 0;

    axis->error_count = 0;
}

void QDecoderAxisState_ProcessAxisPulse(QDecoderAxisState* axis, int a, int b, int index) {
    // if (axis < 0 || axis >= num_axes_) return;
    
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
    
    int from_state = EncodeState(axis->last_channel_a, axis->last_channel_b);
    int to_state = EncodeState(a, b);
    
    // Validate transition
    int direction = GetDirection(from_state, to_state);
    
    if (direction == 0) {
        // Invalid transition detected
        axis->error_count++;
        // Don't update position on invalid transition
    } 
    else if (direction > 0) {
        // Forward direction
        axis->absolute_count++;
    } 
    else {
        // Reverse direction
        axis->absolute_count--;
    }
    
    axis->last_channel_a = a;
    axis->last_channel_b = b;
    axis->last_index = index;
}

int QDecoderAxisState_GetPositionCount(QDecoderAxisState* axis) {
    // if (axis < 0 || axis >= num_axes_) return 0;
    long long count = axis->absolute_count;
    long long positions_per_rev = 4LL * axis->cpr;

    return (int)(count % positions_per_rev);
}

double QDecoderAxisState_GetAngleDeg(QDecoderAxisState* axis) {
    // if (axis < 0 || axis >= num_axes_) return 0.0;
    long long positions_per_rev = 4LL * axis->cpr;
    long long count = axis->absolute_count % positions_per_rev;

    if (count < 0) count += positions_per_rev;

    return ((double)(count) / positions_per_rev) * 360.0;
}

double QDecoderAxisState_GetAngleRad(QDecoderAxisState* axis) {
    // if (axis < 0 || axis >= num_axes_) return 0.0;
    long long positions_per_rev = 4LL * axis->cpr;
    long long count = axis->absolute_count % positions_per_rev;

    if (count < 0) count += positions_per_rev;

    return ((double)(count) / positions_per_rev) * 2.0 * M_PI;
}

int QDecoderAxisState_GetRevolutionCount(QDecoderAxisState* axis) {
    // if (axis < 0 || axis >= num_axes_) return 0;
    long long positions_per_rev = 4LL * axis->cpr;
    return (int)(axis->absolute_count / positions_per_rev);
}

bool QDecoderAxisState_HasErrors(QDecoderAxisState* axis) {
    // if (axis < 0 || axis >= num_axes_) return false;
    return axis->error_count > 0;
}

void QDecoderAxisState_ClearErrors(QDecoderAxisState* axis) {
    // if (axis < 0 || axis >= num_axes_) return;
    axis->error_count = 0;
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

    snprintf(buffer, buffer_size, "%.3f,%d,%d",
             QDecoderAxisState_GetAngleDeg(&decoder->axes[axis]),
             QDecoderAxisState_GetPositionCount(&decoder->axes[axis]),
             QDecoderAxisState_GetRevolutionCount(&decoder->axes[axis]));
}

void QuadratureDecoder_GetFormattedOutputDual(const QuadratureDecoder* decoder, char* buffer, size_t buffer_size) {
    if (decoder->num_axes <= 0) {
        if (buffer_size > 0) {
            buffer[0] = '\0';
        }
        return;
    }

    if (decoder->num_axes == 1) {
        snprintf(buffer, buffer_size, "%.3f,%d,%d",
                 QDecoderAxisState_GetAngleDeg(&decoder->axes[0]),
                 QDecoderAxisState_GetPositionCount(&decoder->axes[0]),
                 QDecoderAxisState_GetRevolutionCount(&decoder->axes[0]));
    }
    else {
        snprintf(buffer, buffer_size, "%.3f,%d,%d,%.3f,%d,%d",
                 QDecoderAxisState_GetAngleDeg(&decoder->axes[0]),
                 QDecoderAxisState_GetPositionCount(&decoder->axes[0]),
                 QDecoderAxisState_GetRevolutionCount(&decoder->axes[0]),
                 QDecoderAxisState_GetAngleDeg(&decoder->axes[1]),
                 QDecoderAxisState_GetPositionCount(&decoder->axes[1]),
                 QDecoderAxisState_GetRevolutionCount(&decoder->axes[1]));
    }
}