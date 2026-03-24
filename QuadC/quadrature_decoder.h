#ifndef QUADRATURE_DECODER_H
#define QUADRATURE_DECODER_H

#include <stdbool.h>

#include "quadrature_common.h"
#include "quadrature_output.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int cpr;                                // counts per revolution       
    int ppr;                                // positions per revolution
    long long count;                        // position count
    long long absolute_count;               // position count (can be negative)

    int last_channel_a;                     // last A signal (0 or 1)
    int last_channel_b;                     // last B signal (0 or 1)
    int last_index;                         // last index signal (0 or 1)

    bool synchronized;                      // synchronized to index

    unsigned long long pulse_count;         // total pulses processed
    unsigned long long error_count;         // invalid transitions
    unsigned long long index_pulse_count;   // index pulses seen
} QDecoderAxisState;

typedef struct {
    int num_axes;
    QDecoderAxisState axes[MAX_AXES];
} QuadratureDecoder;

//============================================================================================
//                          QDecoderAxisState Implementation
//============================================================================================

QDecoderAxisState QDecoderAxisState_ConstructDefault();
QDecoderAxisState QDecoderAxisState_Construct(int cpr);

void QDecoderAxisState_InitializeAtIndex(QDecoderAxisState* axis);

void QDecoderAxisState_Reset(QDecoderAxisState* axis);

void QDecoderAxisState_ProcessAxisPulse(QDecoderAxisState* axis, int a, int b, int index);

long long QDecoderAxisState_GetRevolutionCount(QDecoderAxisState* axis);

long long QDecoderAxisState_GetPositionCount(QDecoderAxisState* axis);

double QDecoderAxisState_GetAngleDeg(QDecoderAxisState* axis);

double QDecoderAxisState_GetAngleRad(QDecoderAxisState* axis);

bool QDecoderAxisState_HasErrors(QDecoderAxisState* axis);

void QDecoderAxisState_ClearErrors(QDecoderAxisState* axis);

//============================================================================================
//                          QuadratureDecoder Implementation
//============================================================================================

QuadratureDecoder QuadratureDecoder_ConstructDefault();
QuadratureDecoder QuadratureDecoder_Construct(int cpr, int num_axes);

void QuadratureDecoder_SetCPR(QuadratureDecoder* decoder, int cpr);

void QuadratureDecoder_SetNumAxes(QuadratureDecoder* decoder, int num_axes);

void QuadratureDecoder_ProcessPulse(QuadratureDecoder* decoder, int axis, int ch_a, int ch_b, int index);

void QuadratureDecoder_ProcessPulseOutput(QuadratureDecoder* decoder, const QuadratureOutput* output);

void QuadratureDecoder_ProcessPulseChannels(QuadratureDecoder* decoder, int ch_a1, int ch_b1, int ch_a2, int ch_b2, int index);

void QuadratureDecoder_GetFormattedOutput(const QuadratureDecoder* decoder, int axis, char* buffer, size_t buffer_size);

void QuadratureDecoder_GetFormattedOutputDual(const QuadratureDecoder* decoder, char* buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif 