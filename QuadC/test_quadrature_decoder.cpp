#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include "quadrature_decoder.hpp"
#include "quadrature_output.hpp"

#define M_PI 3.14159265358979323846
const double ANGLE_TOLERANCE = 0.5;  // degrees
const double RADIAN_TOLERANCE = 0.01;  // radians

// ============================================================================
// TEST GROUP: Initialization and Configuration
// ============================================================================

/*
TEST_CASE("QuadratureDecoder initialization with default parameters", "[init]") {
    QuadratureDecoder qd;
    
    REQUIRE(qd.getNumAxes() == 1);
    REQUIRE(qd.getCPR(0) == 4096);
    REQUIRE(qd.isSynchronized(0) == false);
}

TEST_CASE("QuadratureDecoder initialization with custom CPR", "[init]") {
    QuadratureDecoder qd(2048);
    
    REQUIRE(qd.getCPR(0) == 2048);
}

TEST_CASE("QuadratureDecoder initialization with dual axis", "[init]") {
    QuadratureDecoder qd(4096, 2);
    
    REQUIRE(qd.getNumAxes() == 2);
    REQUIRE(qd.getCPR(0) == 4096);
    REQUIRE(qd.getCPR(1) == 4096);
}

TEST_CASE("QuadratureDecoder CPR bounds checking", "[init]") {
    QuadratureDecoder qd(0, 1);
    REQUIRE(qd.getCPR(0) >= 1);
    
    QuadratureDecoder qd2(99999, 1);
    REQUIRE(qd2.getCPR(0) <= 9000);
}

TEST_CASE("Set CPR for all axes", "[config]") {
    QuadratureDecoder qd(4096, 2);
    qd.setCPR(2048);
    
    REQUIRE(qd.getCPR(0) == 2048);
    REQUIRE(qd.getCPR(1) == 2048);
}

TEST_CASE("Set CPR for specific axis", "[config]") {
    QuadratureDecoder qd(4096, 2);
    qd.setCPRAxis(0, 2048);
    
    REQUIRE(qd.getCPR(0) == 2048);
    REQUIRE(qd.getCPR(1) == 4096);
}

TEST_CASE("Set number of axes", "[config]") {
    QuadratureDecoder qd(4096, 1);
    qd.setNumAxes(2);
    
    REQUIRE(qd.getNumAxes() == 2);
}

// ============================================================================
// TEST GROUP: Initialization at Index
// ============================================================================

TEST_CASE("Initialize at index A=0 B=0", "[index-init]") {
    QuadratureDecoder qd(4096, 1);
    qd.initializeAtIndex(0);
    
    REQUIRE(qd.isSynchronized(0) == true);
    REQUIRE(qd.getPositionCount(0) == 0);
    REQUIRE(qd.getAngle(0) < ANGLE_TOLERANCE);
}

TEST_CASE("Initialize at index sets absolute count to zero", "[index-init]") {
    QuadratureDecoder qd(4096, 1);
    qd.initializeAtIndex(0);
    
    REQUIRE(qd.getAbsoluteCount(0) == 0);
    REQUIRE(qd.getRevolutionCount(0) == 0);
}

TEST_CASE("Initialize dual axis at index", "[index-init]") {
    QuadratureDecoder qd(4096, 2);
    qd.initializeAtIndex(0);
    qd.initializeAtIndex(1);
    
    REQUIRE(qd.isSynchronized(0) == true);
    REQUIRE(qd.isSynchronized(1) == true);
}

// ============================================================================
// TEST GROUP: Reset and State Management
// ============================================================================

TEST_CASE("Reset clears axis state", "[reset]") {
    QuadratureDecoder qd(4096, 1);
    qd.initializeAtIndex(0);
    qd.processPulse(0, 1, 0, 0);  // advance position
    
    qd.reset(0);
    
    REQUIRE(qd.getPositionCount(0) == 0);
    REQUIRE(qd.isSynchronized(0) == false);
    REQUIRE(qd.getPulseCount(0) == 0);
}

TEST_CASE("Reset clears error count", "[reset]") {
    QuadratureDecoder qd(4096, 1);
    qd.initializeAtIndex(0);
    qd.processPulse(0, 1, 1, 0);  // invalid transition from 00
    
    qd.reset(0);
    
    REQUIRE(qd.getErrorCount(0) == 0);
}

TEST_CASE("Clear errors on specific axis", "[reset]") {
    QuadratureDecoder qd(4096, 2);
    qd.initializeAtIndex(0);
    qd.initializeAtIndex(1);
    qd.processPulse(0, 1, 1, 0);  // error on axis 0
    
    qd.clearErrors(0);
    
    REQUIRE(qd.getErrorCount(0) == 0);
}

// ============================================================================
// TEST GROUP: Basic Pulse Processing
// ============================================================================

TEST_CASE("Process single pulse forward", "[pulse]") {
    QuadratureDecoder qd(4, 1);  // small CPR for easy counting
    qd.initializeAtIndex(0);
    
    // Forward transition: 00 -> 10
    qd.processPulse(0, 1, 0, 0);
    
    REQUIRE(qd.getPositionCount(0) == 1);
    REQUIRE(qd.getLastChannelA(0) == 1);
    REQUIRE(qd.getLastChannelB(0) == 0);
}

TEST_CASE("Process single pulse backward", "[pulse]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    // Backward transition: 00 -> 01
    qd.processPulse(0, 0, 1, 0);
    
    REQUIRE(qd.getPositionCount(0) == 4 * 4 - 1);  // wraps around
    REQUIRE(qd.getAbsoluteCount(0) == -1);
}

TEST_CASE("Process repeated same state does nothing", "[pulse]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    qd.processPulse(0, 0, 0, 0);  // same as initial state
    qd.processPulse(0, 0, 0, 0);  // repeat
    
    REQUIRE(qd.getPositionCount(0) == 0);
    REQUIRE(qd.getPulseCount(0) == 2);  // pulse_count increments anyway
}

TEST_CASE("Process full forward quadrature sequence", "[pulse]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    // Full forward cycle: 00 -> 10 -> 11 -> 01 -> 00
    qd.processPulse(0, 1, 0, 0);  // position 1
    qd.processPulse(0, 1, 1, 0);  // position 2
    qd.processPulse(0, 0, 1, 0);  // position 3
    qd.processPulse(0, 0, 0, 0);  // position 0 (wraps)
    
    REQUIRE(qd.getAbsoluteCount(0) == 4);
    REQUIRE(qd.getPositionCount(0) == 0);
}

TEST_CASE("Process full reverse quadrature sequence", "[pulse]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    // Full reverse cycle: 00 -> 01 -> 11 -> 10 -> 00
    qd.processPulse(0, 0, 1, 0);  // position -1
    qd.processPulse(0, 1, 1, 0);  // position -2
    qd.processPulse(0, 1, 0, 0);  // position -3
    qd.processPulse(0, 0, 0, 0);  // position -4
    
    REQUIRE(qd.getAbsoluteCount(0) == -4);
}

// ============================================================================
// TEST GROUP: Error Detection
// ============================================================================

TEST_CASE("Invalid transition detected", "[errors]") {
    QuadratureDecoder qd(4096, 1);
    qd.initializeAtIndex(0);
    
    // Invalid: 00 -> 11 (skips intermediate state)
    qd.processPulse(0, 1, 1, 0);
    
    REQUIRE(qd.getErrorCount(0) == 1);
    REQUIRE(qd.hasErrors(0) == true);
    REQUIRE(qd.getPositionCount(0) == 0);  // position unchanged on error
}

TEST_CASE("Multiple invalid transitions accumulate", "[errors]") {
    QuadratureDecoder qd(4096, 1);
    qd.initializeAtIndex(0);
    
    qd.processPulse(0, 1, 1, 0);  // error
    qd.processPulse(0, 1, 1, 0);  // same state, no change
    qd.processPulse(0, 0, 0, 0);  // error (11 -> 00 invalid backward)
    
    REQUIRE(qd.getErrorCount(0) >= 1);
}

TEST_CASE("Valid transitions do not increment error count", "[errors]") {
    QuadratureDecoder qd(4096, 1);
    qd.initializeAtIndex(0);
    
    qd.processPulse(0, 1, 0, 0);  // valid
    qd.processPulse(0, 1, 1, 0);  // valid
    qd.processPulse(0, 0, 1, 0);  // valid
    
    REQUIRE(qd.getErrorCount(0) == 0);
}

// ============================================================================
// TEST GROUP: Position and Angle Calculation
// ============================================================================

TEST_CASE("Position count wraps at full revolution", "[position]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    long long positions_per_rev = 4LL * qd.getCPR(0);
    
    for (long long i = 0; i < positions_per_rev; ++i) {
        qd.processPulse(0, (i+1) % 4 == 0 ? 1 : 0, (i+1) % 4 >= 2 ? 1 : 0, 0);
    }
    
    REQUIRE(qd.getPositionCount(0) == 0);
    REQUIRE(qd.getAbsoluteCount(0) == positions_per_rev);
    REQUIRE(qd.getRevolutionCount(0) == 1);
}

TEST_CASE("Angle at position 0", "[angle]") {
    QuadratureDecoder qd(4096, 1);
    qd.initializeAtIndex(0);
    
    REQUIRE(qd.getAngle(0) < ANGLE_TOLERANCE);
}

TEST_CASE("Angle at quarter revolution", "[angle]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    long long quarter_positions = qd.getCPR(0);
    
    for (long long i = 0; i < quarter_positions; ++i) {
        int step = i % 4;
        int a = (step >= 1 && step <= 2) ? 1 : 0;
        int b = (step >= 2 && step <= 3) ? 1 : 0;
        qd.processPulse(0, a, b, 0);
    }
    
    double angle = qd.getAngle(0);
    REQUIRE(std::abs(angle - 90.0) < ANGLE_TOLERANCE);
}

TEST_CASE("Angle at half revolution", "[angle]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    long long half_positions = 2LL * qd.getCPR(0);
    
    for (long long i = 0; i < half_positions; ++i) {
        int step = i % 4;
        int a = (step >= 1 && step <= 2) ? 1 : 0;
        int b = (step >= 2 && step <= 3) ? 1 : 0;
        qd.processPulse(0, a, b, 0);
    }
    
    double angle = qd.getAngle(0);
    REQUIRE(std::abs(angle - 180.0) < ANGLE_TOLERANCE);
}

TEST_CASE("Angle at three-quarter revolution", "[angle]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    long long three_quarter_positions = 3LL * qd.getCPR(0);
    
    for (long long i = 0; i < three_quarter_positions; ++i) {
        int step = i % 4;
        int a = (step >= 1 && step <= 2) ? 1 : 0;
        int b = (step >= 2 && step <= 3) ? 1 : 0;
        qd.processPulse(0, a, b, 0);
    }
    
    double angle = qd.getAngle(0);
    REQUIRE(std::abs(angle - 270.0) < ANGLE_TOLERANCE);
}

TEST_CASE("Angle wraps at 360 degrees", "[angle]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    long long positions_per_rev = 4LL * qd.getCPR(0);
    
    for (long long i = 0; i < positions_per_rev; ++i) {
        int step = i % 4;
        int a = (step >= 1 && step <= 2) ? 1 : 0;
        int b = (step >= 2 && step <= 3) ? 1 : 0;
        qd.processPulse(0, a, b, 0);
    }
    
    double angle = qd.getAngle(0);
    REQUIRE(angle < ANGLE_TOLERANCE);
}

TEST_CASE("Angle in radians", "[angle]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    long long half_positions = 2LL * qd.getCPR(0);
    
    for (long long i = 0; i < half_positions; ++i) {
        int step = i % 4;
        int a = (step >= 1 && step <= 2) ? 1 : 0;
        int b = (step >= 2 && step <= 3) ? 1 : 0;
        qd.processPulse(0, a, b, 0);
    }
    
    double angle_rad = qd.getAngleRadians(0);
    REQUIRE(std::abs(angle_rad - M_PI) < RADIAN_TOLERANCE);
}

// ============================================================================
// TEST GROUP: Reverse Motion Detection
// ============================================================================

TEST_CASE("Reverse motion decreases position count", "[reverse]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    // Move forward
    qd.processPulse(0, 1, 0, 0);
    qd.processPulse(0, 1, 1, 0);
    int forward_pos = qd.getPositionCount(0);
    
    // Move backward
    qd.processPulse(0, 0, 1, 0);
    qd.processPulse(0, 0, 0, 0);
    int back_pos = qd.getPositionCount(0);
    
    REQUIRE(back_pos < forward_pos);
}

TEST_CASE("Reverse motion from starting position wraps negative", "[reverse]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    qd.processPulse(0, 0, 1, 0);  // backwards
    
    REQUIRE(qd.getAbsoluteCount(0) == -1);
}

// ============================================================================
// TEST GROUP: Index Pulse Detection
// ============================================================================

TEST_CASE("Index pulse increments counter", "[index-pulse]") {
    QuadratureDecoder qd(4096, 1);
    qd.initializeAtIndex(0);
    
    qd.processPulse(0, 0, 0, 0);  // index low
    qd.processPulse(0, 0, 0, 1);  // index pulse (0->1)
    
    REQUIRE(qd.getIndexPulseCount(0) == 1);
}

TEST_CASE("Index pulse sets synchronized flag", "[index-pulse]") {
    QuadratureDecoder qd(4096, 1);
    
    REQUIRE(qd.isSynchronized(0) == false);
    
    qd.processPulse(0, 0, 0, 0);  // index low
    qd.processPulse(0, 0, 0, 1);  // index pulse
    
    REQUIRE(qd.isSynchronized(0) == true);
}

TEST_CASE("Multiple index pulses", "[index-pulse]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    long long positions_per_rev = 4LL * qd.getCPR(0);
    
    // Full revolution with index at end
    for (long long i = 0; i < positions_per_rev; ++i) {
        int step = i % 4;
        int a = (step >= 1 && step <= 2) ? 1 : 0;
        int b = (step >= 2 && step <= 3) ? 1 : 0;
        int index = (i == positions_per_rev - 1) ? 1 : 0;
        qd.processPulse(0, a, b, index);
    }
    
    REQUIRE(qd.getIndexPulseCount(0) >= 1);
}

// ============================================================================
// TEST GROUP: Source: QuadratureOutput Integration
// ============================================================================

TEST_CASE("Decode QuadratureOutput at 0 degrees", "[integration]") {
    QuadratureOutput qo(4, 1);
    QuadratureDecoder qd(4, 1);
    
    qo.initialize(0.0, 0.0);
    qd.initializeAtIndex(0);
    
    qd.processPulse(qo);
    
    REQUIRE(qd.getAngle(0) < ANGLE_TOLERANCE);
}

TEST_CASE("Decode QuadratureOutput rotating forward", "[integration]") {
    QuadratureOutput qo(4, 1);
    QuadratureDecoder qd(4, 1);
    
    qo.initialize(0.0, 0.0);
    qd.initializeAtIndex(0);
    
    // Rotate through 90 degrees
    for (double angle = 0.0; angle <= 90.0; angle += 1.0) {
        qo.update(angle, 0.0);
        qd.processPulse(qo);
    }
    
    double decoded_angle = qd.getAngle(0);
    REQUIRE(std::abs(decoded_angle - 90.0) < ANGLE_TOLERANCE);
}

TEST_CASE("Decode QuadratureOutput full rotation", "[integration]") {
    QuadratureOutput qo(4, 1);
    QuadratureDecoder qd(4, 1);
    
    qo.initialize(0.0, 0.0);
    qd.initializeAtIndex(0);
    
    // Full 360 degree rotation
    for (double angle = 0.0; angle <= 360.0; angle += 1.0) {
        qo.update(angle, 0.0);
        qd.processPulse(qo);
    }
    
    double decoded_angle = qd.getAngle(0);
    REQUIRE(decoded_angle < ANGLE_TOLERANCE);
    REQUIRE(qd.getRevolutionCount(0) == 1);
}

TEST_CASE("Decode QuadratureOutput oscillating motion", "[integration]") {
    QuadratureOutput qo(4, 1);
    QuadratureDecoder qd(4, 1);
    
    qo.initialize(180.0, 0.0);
    qd.initializeAtIndex(0);
    
    // Oscillate around 180 degrees
    for (int i = 0; i < 10; ++i) {
        qo.update(180.0 + i, 0.0);
        qd.processPulse(qo);
    }
    for (int i = 10; i >= 0; --i) {
        qo.update(180.0 + i, 0.0);
        qd.processPulse(qo);
    }
    
    REQUIRE(qd.isSynchronized(0) == true);
}

// ============================================================================
// TEST GROUP: Dual-Axis Operation
// ============================================================================

TEST_CASE("Dual axis independent decoding", "[dual-axis]") {
    QuadratureOutput qo(4, 2);
    QuadratureDecoder qd(4, 2);
    
    qo.initialize(0.0, 0.0);
    qd.initializeAtIndex(0);
    qd.initializeAtIndex(1);
    
    for (double angle = 0.0; angle <= 90.0; angle += 1.0) {
        qo.update(angle, angle * 2);  // second axis at 2x
        qd.processPulse(qo);
    }
    
    double angle1 = qd.getAngle(0);
    double angle2 = qd.getAngle(1);
    
    REQUIRE(std::abs(angle1 - 90.0) < ANGLE_TOLERANCE);
    REQUIRE(std::abs(angle2 - 180.0) < ANGLE_TOLERANCE);
}

TEST_CASE("Dual axis with different CPR", "[dual-axis]") {
    QuadratureOutput qo(4096, 2);
    QuadratureDecoder qd(4096, 2);
    
    qd.setCPRAxis(0, 2048);
    qd.setCPRAxis(1, 8192);
    
    qo.initialize(0.0, 0.0);
    qd.initializeAtIndex(0);
    qd.initializeAtIndex(1);
    
    for (double angle = 0.0; angle <= 180.0; angle += 1.0) {
        qo.update(angle, angle);
        qd.processPulse(qo);
    }
    
    // Both should be at 180, but with different internal counts
    REQUIRE(std::abs(qd.getAngle(0) - 180.0) < ANGLE_TOLERANCE);
    REQUIRE(std::abs(qd.getAngle(1) - 180.0) < ANGLE_TOLERANCE);
    REQUIRE(qd.getPositionCount(0) != qd.getPositionCount(1));
}

// ============================================================================
// TEST GROUP: Formatted Output
// ============================================================================

TEST_CASE("Get formatted output for single axis", "[format]") {
    QuadratureDecoder qd(4096, 1);
    qd.initializeAtIndex(0);
    
    std::string output = qd.getFormattedOutput(0);
    
    // Should be "angle,position,revolutions"
    int commas = 0;
    for (char c : output) {
        if (c == ',') commas++;
    }
    REQUIRE(commas == 2);
}

TEST_CASE("Get formatted output for dual axis", "[format]") {
    QuadratureDecoder qd(4096, 2);
    qd.initializeAtIndex(0);
    qd.initializeAtIndex(1);
    
    std::string output = qd.getFormattedOutputDual();
    
    // Should be "angle1,pos1,rev1,angle2,pos2,rev2"
    int commas = 0;
    for (char c : output) {
        if (c == ',') commas++;
    }
    REQUIRE(commas == 5);
}

// ============================================================================
// TEST GROUP: State Query
// ============================================================================

TEST_CASE("Get last channel values", "[state]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    qd.processPulse(0, 1, 0, 0);
    
    REQUIRE(qd.getLastChannelA(0) == 1);
    REQUIRE(qd.getLastChannelB(0) == 0);
    REQUIRE(qd.getLastIndex(0) == 0);
}

TEST_CASE("Get quadrature state encoding", "[state]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    REQUIRE(qd.getQuadratureState(0) == 0);  // 00
    
    qd.processPulse(0, 1, 0, 0);
    REQUIRE(qd.getQuadratureState(0) == 2);  // 10
    
    qd.processPulse(0, 1, 1, 0);
    REQUIRE(qd.getQuadratureState(0) == 3);  // 11
    
    qd.processPulse(0, 0, 1, 0);
    REQUIRE(qd.getQuadratureState(0) == 1);  // 01
}

TEST_CASE("Get pulse count", "[state]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    qd.processPulse(0, 0, 0, 0);
    REQUIRE(qd.getPulseCount(0) == 1);
    
    qd.processPulse(0, 1, 0, 0);
    REQUIRE(qd.getPulseCount(0) == 2);
}

// ============================================================================
// TEST GROUP: Edge Cases
// ============================================================================

TEST_CASE("Very small CPR", "[edge-cases]") {
    QuadratureDecoder qd(1, 1);
    qd.initializeAtIndex(0);
    
    qd.processPulse(0, 1, 0, 0);
    
    REQUIRE(qd.isSynchronized(0) == true);
}

TEST_CASE("Large CPR", "[edge-cases]") {
    QuadratureDecoder qd(9000, 1);
    qd.initializeAtIndex(0);
    
    qd.processPulse(0, 1, 0, 0);
    
    REQUIRE(qd.isSynchronized(0) == true);
}

TEST_CASE("Negative position wrapping", "[edge-cases]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    // Go backward from start
    for (int i = 0; i < 20; ++i) {
        qd.processPulse(0, 0, 1, 0);
        qd.processPulse(0, 1, 1, 0);
        qd.processPulse(0, 1, 0, 0);
        qd.processPulse(0, 0, 0, 0);
    }
    
    // Angle should still be in [0, 360)
    double angle = qd.getAngle(0);
    REQUIRE(angle >= 0.0);
    REQUIRE(angle < 360.0);
}

TEST_CASE("Absolute count grows correctly", "[edge-cases]") {
    QuadratureDecoder qd(4, 1);
    qd.initializeAtIndex(0);
    
    for (int i = 0; i < 100; ++i) {
        int step = i % 4;
        int a = (step >= 1 && step <= 2) ? 1 : 0;
        int b = (step >= 2 && step <= 3) ? 1 : 0;
        qd.processPulse(0, a, b, 0);
    }
    
    REQUIRE(qd.getAbsoluteCount(0) == 100);
}
    */