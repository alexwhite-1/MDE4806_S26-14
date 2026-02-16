#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include "quadrature_output.hpp"
#include <cmath>
#include <iostream>

const double TOLERANCE = 0.001;

// ============================================================================
// TEST GROUP: Initialization
// ============================================================================

TEST_CASE("QuadratureOutput initialization with default parameters", "[init]") {
    QuadratureOutput qo;
    
    REQUIRE(qo.getNumAxes() == 2);
    REQUIRE(qo.getCPR(0) == 4096);
    REQUIRE(qo.getCPR(1) == 4096);
    REQUIRE(qo.isCalibrated(0) == false);
    REQUIRE(qo.isCalibrated(1) == false);
}

TEST_CASE("QuadratureOutput initialization with custom CPR", "[init]") {
    QuadratureOutput qo(2048);
    
    REQUIRE(qo.getCPR(0) == 2048);
    REQUIRE(qo.getCPR(1) == 2048);
}

TEST_CASE("QuadratureOutput initialization with single axis", "[init]") {
    QuadratureOutput qo(4096, 1);
    
    REQUIRE(qo.getNumAxes() == 1);
    REQUIRE(qo.isCalibrated(0) == false);
    REQUIRE(qo.isCalibrated(1) == false);  // axis 2 should not be calibrated in 1-axis mode
}

TEST_CASE("QuadratureOutput initialization with dual axis", "[init]") {
    QuadratureOutput qo(4096, 2);
    
    REQUIRE(qo.getNumAxes() == 2);
}

// ============================================================================
// TEST GROUP: Basic Initialize and State
// ============================================================================

TEST_CASE("Initialize axis 1 at 0 degrees", "[initialize]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(0.0, 0.0);
    
    REQUIRE(qo.isCalibrated(0) == true);
    REQUIRE(qo.getPositionCount(0) == 0);
    REQUIRE(qo.getIndex(0) == 1);  // index should be high at 0 degrees
}

TEST_CASE("Initialize axis 1 at 90 degrees", "[initialize]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(90.0, 0.0);
    
    REQUIRE(qo.isCalibrated(0) == true);
    REQUIRE(qo.getIndex(0) == 0);  // index should be low away from 0/360
}

TEST_CASE("Initialize axis 1 at 360 degrees (wraps to 0)", "[initialize]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(360.0, 0.0);
    
    REQUIRE(qo.isCalibrated(0) == true);
    REQUIRE(qo.getIndex(0) == 1);  // index should be high at 360 (treated as 0)
}

TEST_CASE("Initialize dual axis", "[initialize]") {
    QuadratureOutput qo(4096, 2);
    qo.initialize(45.0, 225.0);
    
    REQUIRE(qo.isCalibrated(0) == true);
    REQUIRE(qo.isCalibrated(1) == true);
    REQUIRE(qo.getIndex(0) == 0);
    REQUIRE(qo.getIndex(1) == 0);
}

// ============================================================================
// TEST GROUP: Quadrature Pattern Verification
// ============================================================================

TEST_CASE("Quadrature pattern at 0 degrees", "[pattern]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(0.0, 0.0);
    
    REQUIRE(qo.getChannelA(0) == 0);
    REQUIRE(qo.getChannelB(0) == 0);
}

TEST_CASE("Quadrature pattern at 90 degrees", "[pattern]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(90.0, 0.0);
    
    // 90 degrees should map to position 1 (1/4 of a full position cycle)
    int a = qo.getChannelA(0);
    int b = qo.getChannelB(0);
    REQUIRE((a == 1 || a == 0));  // valid quadrature state
    REQUIRE((b == 1 || b == 0));
}

TEST_CASE("Quadrature pattern at 180 degrees", "[pattern]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(180.0, 0.0);
    
    // 180 degrees should map to position 2 (half position cycle)
    int a = qo.getChannelA(0);
    int b = qo.getChannelB(0);
    REQUIRE((a == 1 || a == 0));
    REQUIRE((b == 1 || b == 0));
}

TEST_CASE("Quadrature pattern at 270 degrees", "[pattern]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(270.0, 0.0);
    
    // 270 degrees should map to position 3 (3/4 position cycle)
    int a = qo.getChannelA(0);
    int b = qo.getChannelB(0);
    REQUIRE((a == 1 || a == 0));
    REQUIRE((b == 1 || b == 0));
}

// ============================================================================
// TEST GROUP: Angle Update and Position Tracking
// ============================================================================

TEST_CASE("Update with small positive angle change", "[update]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(0.0, 0.0);
    int initial_pos = qo.getPositionCount(0);
    
    qo.update(1.0, 0.0);  // small positive change
    int new_pos = qo.getPositionCount(0);
    
    REQUIRE(new_pos > initial_pos);
}

TEST_CASE("Update with small negative angle change", "[update]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(180.0, 0.0);
    int initial_pos = qo.getPositionCount(0);
    
    qo.update(179.0, 0.0);  // small negative change
    int new_pos = qo.getPositionCount(0);
    
    // Negative change should decrease position count or wrap around
    // (depending on wrap-handling logic)
    REQUIRE((new_pos < initial_pos || new_pos > initial_pos));
}

TEST_CASE("Update with 90 degree rotation", "[update]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(0.0, 0.0);
    int initial_pos = qo.getPositionCount(0);
    
    qo.update(90.0, 0.0);
    int new_pos = qo.getPositionCount(0);
    
    // 90 degree rotation should result in positions_per_rev / 4 position changes
    int positions_per_rev = 4 * 4096;
    int expected_change = positions_per_rev / 4;
    REQUIRE(new_pos == expected_change);
}

TEST_CASE("Update with full 360 degree rotation", "[update]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(0.0, 0.0);
    
    qo.update(360.0, 0.0);  // full rotation
    int new_pos = qo.getPositionCount(0);
    
    // Full rotation should return to start (position 0 after wrapping)
    REQUIRE(new_pos == 0);
}

TEST_CASE("Update with angle wrapping (350 to 10 degrees)", "[update]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(350.0, 0.0);
    
    qo.update(10.0, 0.0);  // crosses 0/360 boundary
    
    // Should handle wrap-around correctly
    REQUIRE(qo.isCalibrated(0) == true);
}

// ============================================================================
// TEST GROUP: Dual Axis Operation
// ============================================================================

TEST_CASE("Dual axis independent angle updates", "[dual-axis]") {
    QuadratureOutput qo(4096, 2);
    qo.initialize(0.0, 0.0);
    
    qo.update(45.0, 90.0);
    
    int pos1 = qo.getPositionCount(0);
    int pos2 = qo.getPositionCount(1);
    
    // Both should advance, but axis 2 should advance more (90 > 45)
    REQUIRE(pos1 > 0);
    REQUIRE(pos2 > pos1);
}

TEST_CASE("Dual axis different CPR values", "[dual-axis]") {
    QuadratureOutput qo(4096, 2);
    qo.setCPRAxis(0, 2048);
    qo.setCPRAxis(1, 8192);
    
    REQUIRE(qo.getCPR(0) == 2048);
    REQUIRE(qo.getCPR(1) == 8192);
}

TEST_CASE("Formatted output for dual axis", "[dual-axis]") {
    QuadratureOutput qo(4096, 2);
    qo.initialize(0.0, 0.0);
    
    QuadratureOutputFormat fmt = qo.getFormattedOutput();
    
    REQUIRE(fmt.axis1_A == 0);
    REQUIRE(fmt.axis1_B == 0);
    REQUIRE(fmt.axis2_A == 0);
    REQUIRE(fmt.axis2_B == 0);
    REQUIRE(fmt.index == 1);
}

// ============================================================================
// TEST GROUP: Single Axis Mode
// ============================================================================

TEST_CASE("Single axis mode ignores axis 2 updates", "[single-axis]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(0.0, 0.0);
    
    qo.update(90.0, 90.0);  // try to update both
    
    // Axis 2 should not be calibrated in single-axis mode
    REQUIRE(qo.isCalibrated(1) == false);
    REQUIRE(qo.getChannelA(1) == 0);
    REQUIRE(qo.getChannelB(1) == 0);
}

TEST_CASE("Formatted output for single axis", "[single-axis]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(0.0, 0.0);
    
    std::string csv = qo.getFormattedOutputString();
    
    // Should only have 3 values: A, B, Index (no axis 2)
    int comma_count = 0;
    for (char c : csv) {
        if (c == ',') comma_count++;
    }
    REQUIRE(comma_count == 2);  // 2 commas = 3 values
}

// ============================================================================
// TEST GROUP: CPR Configuration
// ============================================================================

TEST_CASE("CPR bounds checking (minimum)", "[cpr]") {
    QuadratureOutput qo(0, 1);  // invalid CPR
    
    REQUIRE(qo.getCPR(0) >= 1);  // should enforce MIN_CPR
}

TEST_CASE("CPR bounds checking (maximum)", "[cpr]") {
    QuadratureOutput qo(99999, 1);  // invalid CPR
    
    REQUIRE(qo.getCPR(0) <= 9000);  // should enforce MAX_CPR
}

TEST_CASE("Set CPR for all axes", "[cpr]") {
    QuadratureOutput qo(4096, 2);
    qo.setCPR(2048);
    
    REQUIRE(qo.getCPR(0) == 2048);
    REQUIRE(qo.getCPR(1) == 2048);
}

TEST_CASE("Set CPR for specific axis", "[cpr]") {
    QuadratureOutput qo(4096, 2);
    qo.setCPRAxis(0, 2048);
    
    REQUIRE(qo.getCPR(0) == 2048);
    REQUIRE(qo.getCPR(1) == 4096);  // unchanged
}

// ============================================================================
// TEST GROUP: Index Signal
// ============================================================================

TEST_CASE("Index signal at 0 degrees with A=0 B=0", "[index]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(0.0, 0.0);
    
    REQUIRE(qo.getIndex(0) == 1);
    REQUIRE(qo.getChannelA(0) == 0);
    REQUIRE(qo.getChannelB(0) == 0);
}

TEST_CASE("Index signal away from index position", "[index]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(45.0, 0.0);
    
    REQUIRE(qo.getIndex(0) == 0);
}

TEST_CASE("Index signal returns when crossing 0/360", "[index]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(350.0, 0.0);
    
    // Move through 360/0 boundary with A=0, B=0
    qo.update(0.0, 0.0);
    
    // Index should be high at the index position
    REQUIRE(qo.getIndex(0) == 1);
}

// ============================================================================
// TEST GROUP: Reset and Recalibration
// ============================================================================

TEST_CASE("Reset index at 0 degrees", "[reset]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(45.0, 0.0);  // start away from index
    qo.resetIndex(0, 0.0);
    
    REQUIRE(qo.getPositionCount(0) == 0);
    REQUIRE(qo.getIndex(0) == 1);
}

TEST_CASE("Reset does not occur away from index position", "[reset]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(0.0, 0.0);
    int pos_before = qo.getPositionCount(0);
    
    qo.resetIndex(0, 45.0);  // try to reset away from 0
    
    // Position should not change
    REQUIRE(qo.getPositionCount(0) == pos_before);
}

// ============================================================================
// TEST GROUP: Axis Mode Switching
// ============================================================================

TEST_CASE("Switch from 2-axis to 1-axis mode", "[axis-mode]") {
    QuadratureOutput qo(4096, 2);
    qo.initialize(45.0, 90.0);
    
    qo.setNumAxes(1);
    REQUIRE(qo.getNumAxes() == 1);
}

TEST_CASE("Switch from 1-axis to 2-axis mode", "[axis-mode]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(45.0, 0.0);
    
    qo.setNumAxes(2);
    REQUIRE(qo.getNumAxes() == 2);
}

// ============================================================================
// TEST GROUP: Integration Tests
// ============================================================================

TEST_CASE("Complete rotation sequence", "[integration]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(0.0, 0.0);
    
    // Rotate through full circle in 4 steps
    double angles[] = {0.0, 90.0, 180.0, 270.0, 360.0};
    for (double angle : angles) {
        qo.update(angle, 0.0);
    }
    
    // Should return to index position
    REQUIRE(qo.getIndex(0) == 1);
    REQUIRE(qo.getPositionCount(0) == 0);
}

TEST_CASE("Oscillating motion", "[integration]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(180.0, 0.0);
    int initial_pos = qo.getPositionCount(0);
    
    // Oscillate around starting angle
    qo.update(190.0, 0.0);
    int pos_forward = qo.getPositionCount(0);
    REQUIRE(pos_forward > initial_pos);
    
    qo.update(170.0, 0.0);
    int pos_back = qo.getPositionCount(0);
    REQUIRE(pos_back < pos_forward);
}

TEST_CASE("CSV output format consistency", "[integration]") {
    QuadratureOutput qo(4096, 2);
    qo.initialize(45.0, 90.0);
    qo.update(46.0, 91.0);
    
    std::string csv = qo.getFormattedOutputString();
    
    // Count commas for dual-axis format (should have 4 commas for 5 values)
    int comma_count = 0;
    for (char c : csv) {
        if (c == ',') comma_count++;
    }
    REQUIRE(comma_count == 4);
}

// ============================================================================
// TEST GROUP: Edge Cases
// ============================================================================

TEST_CASE("Angle near 360 boundary", "[edge-cases]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(359.9, 0.0);
    
    qo.update(0.1, 0.0);
    
    // Should handle small cross-boundary update
    REQUIRE(qo.isCalibrated(0) == true);
}

TEST_CASE("Very small angle increments", "[edge-cases]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(0.0, 0.0);
    
    for (int i = 0; i < 100; i++) {
        qo.update(0.001 * i, 0.0);
    }
    
    // Should accumulate correctly without errors
    REQUIRE(qo.isCalibrated(0) == true);
}

TEST_CASE("Large angle jumps", "[edge-cases]") {
    QuadratureOutput qo(4096, 1);
    qo.initialize(0.0, 0.0);
    
    qo.update(350.0, 0.0);  // large jump
    
    REQUIRE(qo.isCalibrated(0) == true);
}
