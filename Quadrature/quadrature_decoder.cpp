//Author: Alex White, 2/2/2026
//Version: 1
//
//This file contains the functionality of the quadrature decoding for the S26-14 project.
//Intended functionality: The quadrature output is the chosen method of outputting angle data from the sensors on the device. This method
//of output is not easily read by humans, and so a decoder is needed to take the quadrature output and convert it into a human-readable angle value.
//The angle range is 0 to 360. Mostly intended for use in testing the quadrature encoded output.

#include "quadrature_decoder.hpp"
#include <cmath>
#include <iomanip>
#include <sstream>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Define static const members
const int QuadratureDecoder::MIN_CPR;
const int QuadratureDecoder::MAX_CPR;
const int QuadratureDecoder::MAX_AXES;

// Valid state transitions in quadrature encoding
// Forward: 0->1->3->2->0 (or reverse of this)
// State encoding: State = (A << 1) | B
// State 0: AB=00
// State 1: AB=10
// State 2: AB=11
// State 3: AB=01
const int QuadratureDecoder::VALID_TRANSITIONS[QUADRATURE_STATES][QUADRATURE_STATES] = {
    {0,  -1, 1,  -1},  // from state 0: can go to 1 (fwd) or 3 (rev)
    {1, 0,  -1, 1},  // from state 1: can go to 2 (fwd) or 0 (rev)
    {-1,  1, 0, -1},  // from state 2: can go to 3 (fwd) or 1 (rev)
    {-1, -1,  1, 0}   // from state 3: can go to 0 (fwd) or 2 (rev)
};


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

void QuadratureDecoder::setCPRAxis(int axis, int cpr) {
    if (axis < 0 || axis >= MAX_AXES) return;
    cpr_[axis] = clampCPR_(cpr);
}

int QuadratureDecoder::getCPR(int axis) const {
    if (axis < 0 || axis >= MAX_AXES) return cpr_[0];
    return cpr_[axis];
}

void QuadratureDecoder::setNumAxes(int num_axes) {
    num_axes_ = std::min(std::max(num_axes, 1), MAX_AXES);
}

int QuadratureDecoder::getNumAxes() const {
    return num_axes_;
}

void QuadratureDecoder::initializeAtIndex(int axis) {
    if (axis < 0 || axis >= num_axes_) return;
    state_[axis].absolute_count = 0;
    state_[axis].synchronized = true;
    state_[axis].last_channel_a = 0;
    state_[axis].last_channel_b = 0;
    state_[axis].last_index = 1;
}

void QuadratureDecoder::reset(int axis) {
    if (axis < 0 || axis >= num_axes_) return;
    state_[axis].absolute_count = 0;
    state_[axis].synchronized = false;
    state_[axis].last_channel_a = 0;
    state_[axis].last_channel_b = 0;
    state_[axis].last_index = 0;
    state_[axis].pulse_count = 0;
    state_[axis].index_pulse_count = 0;
    state_[axis].error_count = 0;
}

bool QuadratureDecoder::isSynchronized(int axis) const {
    if (axis < 0 || axis >= num_axes_) return false;
    return state_[axis].synchronized;
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

int QuadratureDecoder::getPositionCount(int axis) const {
    if (axis < 0 || axis >= num_axes_) return 0;
    long long count = state_[axis].absolute_count;
    long long positions_per_rev = 4LL * cpr_[axis];
    return static_cast<int>(count % positions_per_rev);
}

double QuadratureDecoder::getAngle(int axis) const {
    if (axis < 0 || axis >= num_axes_) return 0.0;
    long long positions_per_rev = 4LL * cpr_[axis];
    long long count = state_[axis].absolute_count % positions_per_rev;
    if (count < 0) count += positions_per_rev;
    return (static_cast<double>(count) / positions_per_rev) * 360.0;
}

double QuadratureDecoder::getAngleRadians(int axis) const {
    if (axis < 0 || axis >= num_axes_) return 0.0;
    long long positions_per_rev = 4LL * cpr_[axis];
    long long count = state_[axis].absolute_count % positions_per_rev;
    if (count < 0) count += positions_per_rev;
    return (static_cast<double>(count) / positions_per_rev) * 2.0 * M_PI;
}

long long QuadratureDecoder::getAbsoluteCount(int axis) const {
    if (axis < 0 || axis >= num_axes_) return 0;
    return state_[axis].absolute_count;
}

int QuadratureDecoder::getRevolutionCount(int axis) const {
    if (axis < 0 || axis >= num_axes_) return 0;
    long long positions_per_rev = 4LL * cpr_[axis];
    return static_cast<int>(state_[axis].absolute_count / positions_per_rev);
}

int QuadratureDecoder::getLastChannelA(int axis) const {
    if (axis < 0 || axis >= num_axes_) return 0;
    return state_[axis].last_channel_a;
}

int QuadratureDecoder::getLastChannelB(int axis) const {
    if (axis < 0 || axis >= num_axes_) return 0;
    return state_[axis].last_channel_b;
}

int QuadratureDecoder::getLastIndex(int axis) const {
    if (axis < 0 || axis >= num_axes_) return 0;
    return state_[axis].last_index;
}

int QuadratureDecoder::getQuadratureState(int axis) const {
    if (axis < 0 || axis >= num_axes_) return 0;
    return encodeState(state_[axis].last_channel_a, state_[axis].last_channel_b);
}

unsigned long long QuadratureDecoder::getPulseCount(int axis) const {
    if (axis < 0 || axis >= num_axes_) return 0;
    return state_[axis].pulse_count;
}

unsigned long long QuadratureDecoder::getIndexPulseCount(int axis) const {
    if (axis < 0 || axis >= num_axes_) return 0;
    return state_[axis].index_pulse_count;
}

unsigned long long QuadratureDecoder::getErrorCount(int axis) const {
    if (axis < 0 || axis >= num_axes_) return 0;
    return state_[axis].error_count;
}

bool QuadratureDecoder::hasErrors(int axis) const {
    if (axis < 0 || axis >= num_axes_) return false;
    return state_[axis].error_count > 0;
}

void QuadratureDecoder::clearErrors(int axis) {
    if (axis < 0 || axis >= num_axes_) return;
    state_[axis].error_count = 0;
}

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

int QuadratureDecoder::encodeState(int a, int b) {
    return (a << 1) | b;
}

int QuadratureDecoder::getDirection(int from, int to) {
    if (from < 0 || from >= QUADRATURE_STATES || to < 0 || to >= QUADRATURE_STATES) {
        return 0;
    }
    return VALID_TRANSITIONS[from][to];
}

bool QuadratureDecoder::isValidTransition(int from, int to) {
    int direction = getDirection(from, to);
    return direction != 0;
}

void QuadratureDecoder::processAxisPulse_(int axis, int a, int b, int index) {
    if (axis < 0 || axis >= num_axes_) return;
    
    AxisState& s = state_[axis];
    s.pulse_count++;
    
    // Check for index pulse (transition from 0 to 1)
    if (index == 1 && s.last_index == 0) {
        s.index_pulse_count++;
        s.synchronized = true;
        // Optionally reset position at index
        // s.absolute_count = 0;
    }
    
    // Only process quadrature if state actually changed
    if (a == s.last_channel_a && b == s.last_channel_b) {
        return;  // No change, skip
    }
    
    int from_state = encodeState(s.last_channel_a, s.last_channel_b);
    int to_state = encodeState(a, b);
    
    // Validate transition
    int direction = getDirection(from_state, to_state);
    
    if (direction == 0) {
        // Invalid transition detected
        s.error_count++;
        // Don't update position on invalid transition
    } else if (direction > 0) {
        // Forward direction
        s.absolute_count++;
    } else {
        // Reverse direction
        s.absolute_count--;
    }
    
    s.last_channel_a = a;
    s.last_channel_b = b;
    s.last_index = index;
}

int QuadratureDecoder::clampCPR_(int cpr) {
    return std::max(MIN_CPR, std::min(cpr, MAX_CPR));
}