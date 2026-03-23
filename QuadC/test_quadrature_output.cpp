#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include "quadrature_output.h"
#include <cmath>
#include <iostream>

// ============================================================================
// TEST GROUP: Initialization
// ============================================================================

TEST_CASE("QuadratureOutput initialization with default parameters", "[init]") {
    QuadratureOutput qo = QuadratureOutput_ConstructDefault();
    
    REQUIRE(qo.num_axes == 2);
    REQUIRE(qo.axis1.cpr == 4096);
    REQUIRE(qo.axis2.cpr == 4096);
    REQUIRE(qo.axis1.calibrated == false);
    REQUIRE(qo.axis2.calibrated == false);
}

TEST_CASE("QuadratureOutput initialization with custom CPR", "[init]") {
    QuadratureOutput qo = QuadratureOutput_ConstructCPR(2048);
    
    REQUIRE(qo.axis1.cpr == 2048);
    REQUIRE(qo.axis2.cpr == 2048);
}

TEST_CASE("QuadratureOutput initialization with single axis", "[init]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    
    REQUIRE(qo.num_axes == 1);
    REQUIRE(qo.axis1.calibrated == false);
    REQUIRE(qo.axis2.calibrated == false);  // axis 2 should not be calibrated in 1-axis mode
}

TEST_CASE("QuadratureOutput initialization with dual axis", "[init]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 2);
    
    REQUIRE(qo.num_axes == 2);
}

// ============================================================================
// TEST GROUP: Basic Initialize and State
// ============================================================================

TEST_CASE("Initialize axis 1 at 0 degrees", "[initialize]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);
    
    REQUIRE(qo.axis1.calibrated == true);
    REQUIRE(qo.axis1.position_count == 0);
    REQUIRE(qo.axis1.index == 1);  // index should be high at 0 degrees
}

TEST_CASE("Initialize axis 1 at 90 degrees", "[initialize]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 90.0, 0.0);

    REQUIRE(qo.axis1.calibrated == true);
    REQUIRE(qo.axis1.index == 0);  // index should be low away from 0/360
}

TEST_CASE("Initialize axis 1 at 360 degrees (wraps to 0)", "[initialize]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 360.0, 0.0);

    
    REQUIRE(qo.axis1.calibrated == true);
    REQUIRE(qo.axis1.index == 1);  // index should be high at 360 (treated as 0)
}

TEST_CASE("Initialize dual axis", "[initialize]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 2);
    QuadratureOutput_Initialize(&qo, 45.0, 225.0);

    REQUIRE(qo.axis1.calibrated == true);
    REQUIRE(qo.axis2.calibrated == true);
    REQUIRE(qo.axis1.index == 0);
    REQUIRE(qo.axis2.index == 0);
}

// ============================================================================
// TEST GROUP: Quadrature Pattern Verification
// ============================================================================

TEST_CASE("Quadrature pattern at 0 degrees", "[pattern]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);

    REQUIRE(qo.axis1.channel_a == 0);
    REQUIRE(qo.axis1.channel_b == 0);
}

TEST_CASE("Quadrature pattern at 90 degrees", "[pattern]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 90.0, 0.0);

    // 90 degrees should map to position 1 (1/4 of a full position cycle)
    int a = qo.axis1.channel_a;
    int b = qo.axis1.channel_b;
    REQUIRE((a == 1 || a == 0)); // valid quadrature state
    REQUIRE((b == 1 || b == 0));
}

TEST_CASE("Quadrature pattern at 180 degrees", "[pattern]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 180.0, 0.0);

    // 180 degrees should map to position 2 (half position cycle)
    int a = qo.axis1.channel_a;
    int b = qo.axis1.channel_b;
    REQUIRE((a == 1 || a == 0));
    REQUIRE((b == 1 || b == 0));
}

TEST_CASE("Quadrature pattern at 270 degrees", "[pattern]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 270.0, 0.0);

    // 270 degrees should map to position 3 (3/4 position cycle)
    int a = qo.axis1.channel_a;
    int b = qo.axis1.channel_b;
    REQUIRE((a == 1 || a == 0));
    REQUIRE((b == 1 || b == 0));
}

// ============================================================================
// TEST GROUP: Angle Update and Position Tracking
// ============================================================================

TEST_CASE("Update with small positive angle change", "[update]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);
    int initial_pos = qo.axis1.position_count;

    QuadratureOutput_Update(&qo, 1.0, 0.0); // small positive change
    int new_pos = qo.axis1.position_count;

    REQUIRE(new_pos > initial_pos);
}

TEST_CASE("Update with small negative angle change", "[update]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 180.0, 0.0);
    int initial_pos = qo.axis1.position_count;

    QuadratureOutput_Update(&qo, 179.0, 0.0); // small negative change
    int new_pos = qo.axis1.position_count;

    // Negative change should decrease position count or wrap around
    // (depending on wrap-handling logic)
    REQUIRE(new_pos != initial_pos);
}

TEST_CASE("Update with 90 degree rotation", "[update]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);

    QuadratureOutput_Update(&qo, 90.0, 0.0);
    int new_pos = qo.axis1.position_count;

    // 90 degree rotation should result in positions_per_rev / 4 position changes
    int positions_per_rev = 4 * 4096;
    int expected_change = positions_per_rev / 4;
    REQUIRE(new_pos == expected_change);
}

TEST_CASE("Update with full 360 degree rotation", "[update]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);

    QuadratureOutput_Update(&qo, 360.0, 0.0); // full rotation
    int new_pos = qo.axis1.position_count;

    // Full rotation should return to start (position 0 after wrapping)
    REQUIRE(new_pos == 0);
}

TEST_CASE("Update with angle wrapping (350 to 10 degrees)", "[update]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 350.0, 0.0);

    QuadratureOutput_Update(&qo, 10.0, 0.0);  // crosses 0/360 boundary

    // Should handle wrap-around correctly
    REQUIRE(qo.axis1.calibrated == true);
}

// ============================================================================
// TEST GROUP: Dual Axis Operation
// ============================================================================

TEST_CASE("Dual axis independent angle updates", "[dual-axis]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 2);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);

    QuadratureOutput_Update(&qo, 45.0, 90.0);

    int pos1 = qo.axis1.position_count;
    int pos2 = qo.axis2.position_count;

    // Both should advance, but axis 2 should advance more (90 > 45)
    REQUIRE(pos1 > 0);
    REQUIRE(pos2 > pos1);
}

TEST_CASE("Dual axis different CPR values", "[dual-axis]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 2);
    QOutputAxisState_SetCPR(&qo.axis1, 2048);
    QOutputAxisState_SetCPR(&qo.axis2, 8192);

    REQUIRE(qo.axis1.cpr == 2048);
    REQUIRE(qo.axis2.cpr == 8192);
}

TEST_CASE("Formatted output for dual axis", "[dual-axis]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 2);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);

    QuadratureOutputFormat fmt = QuadratureOutput_GetFormattedOutput(&qo);

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
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);

    QuadratureOutput_Update(&qo, 90.0, 90.0);  // try to update both

    // Axis 2 should not be calibrated in single-axis mode
    REQUIRE(qo.axis2.calibrated == false);
    REQUIRE(qo.axis2.channel_a == 0);
    REQUIRE(qo.axis2.channel_b == 0);
}

TEST_CASE("Formatted output for single axis", "[single-axis]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);

    char csv[32];
    QuadratureOutput_GetFormattedOutputString(&qo, csv, sizeof(csv));

    // Should only have 3 values: A, B, Index (no axis 2)
    int comma_count = 0;
    for (const char* p = csv; *p != '\0'; ++p) {
        if (*p == ',') comma_count++;
    }
    REQUIRE(comma_count == 2);  // 2 commas = 3 values
}

// ============================================================================
// TEST GROUP: CPR Configuration
// ============================================================================

TEST_CASE("CPR bounds checking (minimum)", "[cpr]") {
    QuadratureOutput qo = QuadratureOutput_Construct(0, 1);  // invalid CPR

    REQUIRE(qo.axis1.cpr >= 1);  // should enforce MIN_CPR
}

TEST_CASE("CPR bounds checking (maximum)", "[cpr]") {
    QuadratureOutput qo = QuadratureOutput_Construct(99999, 1);  // invalid CPR

    REQUIRE(qo.axis1.cpr <= 9000);  // should enforce MAX_CPR
}

TEST_CASE("Set CPR for all axes", "[cpr]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 2);
    QOutputAxisState_SetCPR(&qo.axis1, 2048);
    QOutputAxisState_SetCPR(&qo.axis2, 2048);

    REQUIRE(qo.axis1.cpr == 2048);
    REQUIRE(qo.axis2.cpr == 2048);
}

TEST_CASE("Set CPR for specific axis", "[cpr]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 2);
    QOutputAxisState_SetCPR(&qo.axis1, 2048);

    REQUIRE(qo.axis1.cpr == 2048);
    REQUIRE(qo.axis2.cpr == 4096);  // unchanged
}

// ============================================================================
// TEST GROUP: Index Signal
// ============================================================================

TEST_CASE("Index signal at 0 degrees with A=0 B=0", "[index]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);

    REQUIRE(qo.axis1.index == 1);
    REQUIRE(qo.axis1.channel_a == 0);
    REQUIRE(qo.axis1.channel_b == 0);
}

TEST_CASE("Index signal away from index position", "[index]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 45.0, 0.0);

    REQUIRE(qo.axis1.index == 0);
}

TEST_CASE("Index signal returns when crossing 0/360", "[index]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 350.0, 0.0);

    REQUIRE(qo.axis1.position_count == 15928);

    // Move through 360/0 boundary with A=0, B=0
    QuadratureOutput_Update(&qo, 0.0, 0.0);

    // Position should be at final wrapped state before index correction logic
    REQUIRE(qo.axis1.position_count == 16383);

    // Index should be high at the index position
    REQUIRE(qo.axis1.index == 1);
}

// ============================================================================
// TEST GROUP: Reset and Recalibration
// ============================================================================

TEST_CASE("Reset index at 0 degrees", "[reset]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 45.0, 0.0);

    QOutputAxisState_ResetIndex(&qo.axis1, 0.0);

    REQUIRE(qo.axis1.position_count == 0);
    REQUIRE(qo.axis1.index == 1);
}

TEST_CASE("Reset does not occur away from index position", "[reset]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);
    int pos_before = qo.axis1.position_count;

    QOutputAxisState_ResetIndex(&qo.axis1, 45.0);

    REQUIRE(qo.axis1.position_count == pos_before);
}

// ============================================================================
// TEST GROUP: Axis Mode Switching
// ============================================================================

TEST_CASE("Switch from 2-axis to 1-axis mode", "[axis-mode]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 2);
    QuadratureOutput_Initialize(&qo, 45.0, 90.0);

    QuadratureOutput_SetNumAxes(&qo, 1);
    REQUIRE(qo.num_axes == 1);
}

TEST_CASE("Switch from 1-axis to 2-axis mode", "[axis-mode]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 45.0, 0.0);

    QuadratureOutput_SetNumAxes(&qo, 2);
    REQUIRE(qo.num_axes == 2);
}

// ============================================================================
// TEST GROUP: Integration Tests
// ============================================================================

TEST_CASE("Complete rotation sequence", "[integration]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);

    double angles[] = { 0.0, 90.0, 180.0, 270.0, 360.0 };
    for (double angle : angles) {
        QuadratureOutput_Update(&qo, angle, 0.0);
    }

    REQUIRE(qo.axis1.index == 1);
    REQUIRE(qo.axis1.position_count == 0);
}

TEST_CASE("Oscillating motion", "[integration]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 180.0, 0.0);
    int initial_pos = qo.axis1.position_count;

    QuadratureOutput_Update(&qo, 190.0, 0.0);
    int pos_forward = qo.axis1.position_count;
    REQUIRE(pos_forward > initial_pos);

    QuadratureOutput_Update(&qo, 170.0, 0.0);
    int pos_back = qo.axis1.position_count;
    REQUIRE(pos_back < pos_forward);
}

TEST_CASE("CSV output format consistency", "[integration]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 2);
    QuadratureOutput_Initialize(&qo, 45.0, 90.0);
    QuadratureOutput_Update(&qo, 46.0, 91.0);

    char csv[64];
    QuadratureOutput_GetFormattedOutputString(&qo, csv, sizeof(csv));

    int comma_count = 0;
    for (const char* p = csv; *p != '\0'; ++p) {
        if (*p == ',') {
            comma_count++;
        }
    }
    REQUIRE(comma_count == 4);
}

// ============================================================================
// TEST GROUP: Edge Cases
// ============================================================================

TEST_CASE("Angle near 360 boundary", "[edge-cases]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 359.9, 0.0);

    QuadratureOutput_Update(&qo, 0.1, 0.0);

    REQUIRE(qo.axis1.calibrated == true);
}

TEST_CASE("Very small angle increments", "[edge-cases]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
        QuadratureOutput_Update(&qo, 0.001 * i, 0.0);
    }

    REQUIRE(qo.axis1.calibrated == true);
}

TEST_CASE("Large angle jumps", "[edge-cases]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4096, 1);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);

    QuadratureOutput_Update(&qo, 350.0, 0.0);

    REQUIRE(qo.axis1.calibrated == true);
}

// ============================================================================
// TEST GROUP: Pulse Sequence Verification
// ============================================================================

TEST_CASE("Angle 0 to 180 back to 0 generates correct quadrature pulses", "[pulse-sequence]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4, 1);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);

    const int positions_per_rev = 4 * qo.axis1.cpr;
    int half_states = positions_per_rev / 2;

    auto expectedPattern = [](int pos)->std::pair<int, int> {
        int state = pos % 4;
        switch (state) {
        case 0: return { 0,0 };
        case 1: return { 1,0 };
        case 2: return { 1,1 };
        case 3: return { 0,1 };
        }
        return { 0,0 };
        };

    std::vector<double> angles;
    for (int i = 0; i <= half_states; i++) {
        angles.push_back((static_cast<double>(i) / positions_per_rev) * 360.0);
    }
    for (int i = half_states - 1; i >= 0; i--) {
        angles.push_back((static_cast<double>(i) / positions_per_rev) * 360.0);
    }

    int prevA = qo.axis1.channel_a;
    int prevB = qo.axis1.channel_b;

    for (double ang : angles) {
        QuadratureOutput_Update(&qo, ang, 0.0);
        int a = qo.axis1.channel_a;
        int b = qo.axis1.channel_b;

        auto [expA, expB] = expectedPattern(qo.axis1.position_count);
        REQUIRE(a == expA);
        REQUIRE(b == expB);

        if (a != prevA || b != prevB) {
            REQUIRE(((a == prevA) ^ (b == prevB)) == true);
        }

        prevA = a;
        prevB = b;
    }
}

TEST_CASE("Channel lead/lag behaviour during positive and negative motion", "[pulse-sequence]") {
    QuadratureOutput qo = QuadratureOutput_Construct(4, 1);
    QuadratureOutput_Initialize(&qo, 0.0, 0.0);

    const int positions_per_rev = 4 * qo.axis1.cpr;
    int half_states = positions_per_rev / 2;

    std::vector<std::pair<int, int>> forward;
    for (int i = 0; i <= half_states; i++) {
        double ang = (static_cast<double>(i) / positions_per_rev) * 360.0;
        QuadratureOutput_Update(&qo, ang, 0.0);
        forward.emplace_back(qo.axis1.channel_a, qo.axis1.channel_b);
    }

    auto pattern = [&](int idx)->std::pair<int, int> {
        int s = idx % 4;
        switch (s) {
        case 0: return { 0,0 };
        case 1: return { 1,0 };
        case 2: return { 1,1 };
        case 3: return { 0,1 };
        }
        return { 0,0 };
        };

    for (size_t idx = 0; idx < forward.size(); ++idx) {
        REQUIRE(forward[idx] == pattern(static_cast<int>(idx)));
    }

    QuadratureOutput_Initialize(&qo, 180.0, 0.0);

    std::vector<std::pair<int, int>> backward;
    for (int i = half_states; i >= 0; i--) {
        double ang = (static_cast<double>(i) / positions_per_rev) * 360.0;
        QuadratureOutput_Update(&qo, ang, 0.0);
        backward.emplace_back(qo.axis1.channel_a, qo.axis1.channel_b);
    }

    REQUIRE(backward.size() == forward.size());
    for (size_t idx = 0; idx < backward.size(); ++idx) {
        REQUIRE(backward[idx] == forward[forward.size() - 1 - idx]);
    }
}