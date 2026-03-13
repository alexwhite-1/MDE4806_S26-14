#include "quadrature_decoder.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Macros
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

static const int MIN_CPR = 1;
static const int MAX_CPR = 9000;
static const int MAX_AXES = 2;
static const int QUADRATURE_STATES = 4;

static const int VALID_TRANSITIONS[4][4] = {
    {0, -1,  1, -1},
    {1,  0, -1,  1},
    {-1, 1,  0, -1},
    {-1,-1,  1,  0}
};

static int EncodeState(int a, int b);
static int ClampCPR(int cpr);
static int GetDirection(int from, int to);

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

/*
QuadratureDecoder::QuadratureDecoder(int cpr, int num_axes)
    : num_axes_(std::min(num_axes, MAX_AXES)) {
    for (int i = 0; i < MAX_AXES; ++i) {
        cpr_[i] = clampCPR_(cpr);
        state_[i] = {0, 0, 0, 0, false, 0, 0, 0};
    }
}

void QuadratureDecoder::setCPR(int cpr) {
    int clamped = clampCPR_(cpr);
    for (int i = 0; i < num_axes_; ++i) {
        cpr_[i] = clamped;
    }
}

void QuadratureDecoder::setNumAxes(int num_axes) {
    num_axes_ = std::min(std::max(num_axes, 1), MAX_AXES);
}

void QuadratureDecoder::processPulse(const QuadratureOutput& output) {
    if (num_axes_ == 1) {
        processAxisPulse_(0, output.getChannelA(0), output.getChannelB(0), 
                         output.getIndex(0));
    } else if (num_axes_ == 2) {
        processAxisPulse_(0, output.getChannelA(0), output.getChannelB(0), 
                         output.getIndex(0));
        processAxisPulse_(1, output.getChannelA(1), output.getChannelB(1), 
                         output.getIndex(1));
    }
}

void QuadratureDecoder::processPulse(int axis, int channel_a, 
                                      int channel_b, int index_signal) {
    if (axis < 0 || axis >= num_axes_) return;
    processAxisPulse_(axis, channel_a, channel_b, index_signal);
}

void QuadratureDecoder::processPulse(int channel_a_1, int channel_b_1,
                                      int channel_a_2, int channel_b_2, 
                                      int index_signal) {
    if (num_axes_ >= 1) {
        processAxisPulse_(0, channel_a_1, channel_b_1, index_signal);
    }
    if (num_axes_ >= 2) {
        processAxisPulse_(1, channel_a_2, channel_b_2, index_signal);
    }
}

*/

/*

std::string QuadratureDecoder::getFormattedOutput(int axis) const {
    if (axis < 0 || axis >= num_axes_) return "";
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << getAngle(axis) << ","
        << getPositionCount(axis) << ","
        << getRevolutionCount(axis);
    return oss.str();
}

std::string QuadratureDecoder::getFormattedOutputDual() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    if (num_axes_ >= 1) {
        oss << getAngle(0) << "," << getPositionCount(0) << "," 
            << getRevolutionCount(0);
    }
    if (num_axes_ >= 2) {
        oss << "," << getAngle(1) << "," << getPositionCount(1) << "," 
            << getRevolutionCount(1);
    }
    return oss.str();
}
*/