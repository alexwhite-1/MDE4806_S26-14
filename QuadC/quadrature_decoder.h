#ifndef QUADRATURE_DECODER_H
#define QUADRATURE_DECODER_H

#include <stdbool.h>

typedef struct {
    int cpr;
    long long absolute_count;               // position count (can be negative)

    int last_channel_a;                     // last A signal (0 or 1)
    int last_channel_b;                     // last B signal (0 or 1)
    int last_index;                         // last index signal (0 or 1)

    bool synchronized;                      // synchronized to index

    unsigned long long pulse_count;         // total pulses processed
    unsigned long long index_pulse_count;   // index pulses seen
    unsigned long long error_count;         // invalid transitions
} QDecoderAxisState;

typedef struct {
    int placeholder;
} QuadratureDecoder;

QDecoderAxisState QDecoderAxisState_ConstructDefault();
QDecoderAxisState QDecoderAxisState_Construct(int cpr);

void QDecoderAxisState_InitializeAtIndex(QDecoderAxisState* axis);

void QDecoderAxisState_Reset(QDecoderAxisState* axis);

void QDecoderAxisState_ProcessAxisPulse(QDecoderAxisState* axis, int a, int b, int index);

int QDecoderAxisState_GetPositionCount(QDecoderAxisState* axis);

double QDecoderAxisState_GetAngleDeg(QDecoderAxisState* axis);

double QDecoderAxisState_GetAngleRad(QDecoderAxisState* axis);

int QDecoderAxisState_GetRevolutionCount(QDecoderAxisState* axis);

bool QDecoderAxisState_HasErrors(QDecoderAxisState* axis);

void QDecoderAxisState_ClearErrors(QDecoderAxisState* axis);

#endif 