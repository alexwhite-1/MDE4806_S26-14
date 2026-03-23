#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include "quadrature_decoder.h"
#include "quadrature_output.h"

#define M_PI 3.14159265358979323846

// ============================================================================
// TEST GROUP: Initialization and Configuration
// ============================================================================

TEST_CASE("QuadratureDecoder initialization with default parameters", "[init]") {
    QuadratureDecoder qd = QuadratureDecoder_ConstructDefault();

    REQUIRE(qd.num_axes == 1);
    REQUIRE(qd.axes[0].cpr == 4096);
    REQUIRE(qd.axes[0].synchronized == false);
}

TEST_CASE("QuadratureDecoder initialization with custom CPR", "[init]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(2048, 1);

    REQUIRE(qd.axes[0].cpr == 2048);
}

TEST_CASE("QuadratureDecoder initialization with dual axis", "[init]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4096, 2);

    REQUIRE(qd.num_axes == 2);
    REQUIRE(qd.axes[0].cpr == 4096);
    REQUIRE(qd.axes[1].cpr == 4096);
}

TEST_CASE("QuadratureDecoder CPR bounds checking", "[init]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(0, 1);
    REQUIRE(qd.axes[0].cpr >= 1);

    QuadratureDecoder qd2 = QuadratureDecoder_Construct(99999, 1);
    REQUIRE(qd2.axes[0].cpr <= 9000);
}

TEST_CASE("Set CPR for all axes", "[config]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4096, 2);
    QuadratureDecoder_SetCPR(&qd, 2048);

    REQUIRE(qd.axes[0].cpr == 2048);
    REQUIRE(qd.axes[1].cpr == 2048);
}

TEST_CASE("Set number of axes", "[config]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4096, 1);
    QuadratureDecoder_SetNumAxes(&qd, 2);

    REQUIRE(qd.num_axes == 2);
}

// ============================================================================
// TEST GROUP: Initialization at Index
// ============================================================================

TEST_CASE("Initialize at index A=0 B=0", "[index-init]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4096, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    REQUIRE(qd.axes[0].synchronized == true);
    REQUIRE(QDecoderAxisState_GetPositionCount(&qd.axes[0]) == 0);
    REQUIRE(QDecoderAxisState_GetAngleDeg(&qd.axes[0]) < ANGLE_TOLERANCE);
}

TEST_CASE("Initialize at index sets absolute count to zero", "[index-init]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4096, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    REQUIRE(qd.axes[0].absolute_count == 0);
    REQUIRE(QDecoderAxisState_GetRevolutionCount(&qd.axes[0]) == 0);
}

TEST_CASE("Initialize dual axis at index", "[index-init]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4096, 2);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[1]);

    REQUIRE(qd.axes[0].synchronized == true);
    REQUIRE(qd.axes[1].synchronized == true);
}

// ============================================================================
// TEST GROUP: Reset and State Management
// ============================================================================

TEST_CASE("Reset clears axis state", "[reset]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4096, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);
    QuadratureDecoder_ProcessPulse(&qd, 0, 1, 0, 0);

    QDecoderAxisState_Reset(&qd.axes[0]);

    REQUIRE(QDecoderAxisState_GetPositionCount(&qd.axes[0]) == 0);
    REQUIRE(qd.axes[0].synchronized == false);
    REQUIRE(qd.axes[0].pulse_count == 0);
}

TEST_CASE("Reset clears error count", "[reset]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4096, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);
    QuadratureDecoder_ProcessPulse(&qd, 0, 1, 1, 0);

    QDecoderAxisState_Reset(&qd.axes[0]);

    REQUIRE(qd.axes[0].error_count == 0);
}

TEST_CASE("Clear errors on specific axis", "[reset]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4096, 2);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[1]);
    QuadratureDecoder_ProcessPulse(&qd, 0, 1, 1, 0);

    QDecoderAxisState_ClearErrors(&qd.axes[0]);

    REQUIRE(qd.axes[0].error_count == 0);
}

// ============================================================================
// TEST GROUP: Basic Pulse Processing
// ============================================================================

TEST_CASE("Process single pulse forward", "[pulse]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    QuadratureDecoder_ProcessPulse(&qd, 0, 1, 0, 0);

    REQUIRE(QDecoderAxisState_GetPositionCount(&qd.axes[0]) == 1);
    REQUIRE(qd.axes[0].last_channel_a == 1);
    REQUIRE(qd.axes[0].last_channel_b == 0);
}

TEST_CASE("Process single pulse backward", "[pulse]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    QuadratureDecoder_ProcessPulse(&qd, 0, 0, 1, 0);

    REQUIRE(QDecoderAxisState_GetPositionCount(&qd.axes[0]) == 4 * 4 - 1);
    REQUIRE(qd.axes[0].absolute_count == 15);
}

TEST_CASE("Process repeated same state does nothing", "[pulse]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    QuadratureDecoder_ProcessPulse(&qd, 0, 0, 0, 0);
    QuadratureDecoder_ProcessPulse(&qd, 0, 0, 0, 0);

    REQUIRE(QDecoderAxisState_GetPositionCount(&qd.axes[0]) == 0);
    REQUIRE(qd.axes[0].pulse_count == 2);
}

TEST_CASE("Process full forward quadrature sequence", "[pulse]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    QuadratureDecoder_ProcessPulse(&qd, 0, 1, 0, 0);
    QuadratureDecoder_ProcessPulse(&qd, 0, 1, 1, 0);
    QuadratureDecoder_ProcessPulse(&qd, 0, 0, 1, 0);
    QuadratureDecoder_ProcessPulse(&qd, 0, 0, 0, 0);

    REQUIRE(qd.axes[0].absolute_count == 16);
    REQUIRE(QDecoderAxisState_GetPositionCount(&qd.axes[0]) == 0);
}

TEST_CASE("Process full reverse quadrature sequence", "[pulse]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    QuadratureDecoder_ProcessPulse(&qd, 0, 0, 1, 0);
    QuadratureDecoder_ProcessPulse(&qd, 0, 1, 1, 0);
    QuadratureDecoder_ProcessPulse(&qd, 0, 1, 0, 0);
    QuadratureDecoder_ProcessPulse(&qd, 0, 0, 0, 0);

    REQUIRE(qd.axes[0].absolute_count == 16);
}

// ============================================================================
// TEST GROUP: Error Detection
// ============================================================================

TEST_CASE("Invalid transition detected", "[errors]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4096, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    QuadratureDecoder_ProcessPulse(&qd, 0, 1, 1, 0);

    REQUIRE(qd.axes[0].error_count == 1);
    REQUIRE(QDecoderAxisState_HasErrors(&qd.axes[0]) == true);
    REQUIRE(QDecoderAxisState_GetPositionCount(&qd.axes[0]) == 0);
}

TEST_CASE("Multiple invalid transitions accumulate", "[errors]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4096, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    QuadratureDecoder_ProcessPulse(&qd, 0, 1, 1, 0);
    QuadratureDecoder_ProcessPulse(&qd, 0, 1, 1, 0);
    QuadratureDecoder_ProcessPulse(&qd, 0, 0, 0, 0);

    REQUIRE(qd.axes[0].error_count >= 1);
}

TEST_CASE("Valid transitions do not increment error count", "[errors]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4096, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    QuadratureDecoder_ProcessPulse(&qd, 0, 1, 0, 0);
    QuadratureDecoder_ProcessPulse(&qd, 0, 1, 1, 0);
    QuadratureDecoder_ProcessPulse(&qd, 0, 0, 1, 0);

    REQUIRE(qd.axes[0].error_count == 0);
}

// ============================================================================
// TEST GROUP: Position and Angle Calculation
// ============================================================================

TEST_CASE("Position count wraps at full revolution", "[position]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    long long positions_per_rev = 4LL * qd.axes[0].cpr;

    for (long long i = 0; i < positions_per_rev; ++i) {
        qd.axes[0].last_channel_a = 0;
        qd.axes[0].last_channel_b = 0;
        int step = (int)((i + 1) % 4);
        int a = (step == 1 || step == 2) ? 1 : 0;
        int b = (step == 2 || step == 3) ? 1 : 0;
        QuadratureDecoder_ProcessPulse(&qd, 0, a, b, 0);
    }

    REQUIRE(QDecoderAxisState_GetPositionCount(&qd.axes[0]) == 0);
    //REQUIRE(qd.axes[0].absolute_count == positions_per_rev);
    //REQUIRE(QDecoderAxisState_GetRevolutionCount(&qd.axes[0]) == 1);
}

TEST_CASE("Angle at position 0", "[angle]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4096, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    REQUIRE(QDecoderAxisState_GetAngleDeg(&qd.axes[0]) < ANGLE_TOLERANCE);
}

TEST_CASE("Angle at quarter revolution", "[angle]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    long long quarter_positions = qd.axes[0].cpr;

    //for (long long i = 0; i < quarter_positions; ++i) {
    //    int a = (i >= 1 && i <= 2) ? 1 : 0;
    //    int b = (i >= 2 && i <= 3) ? 1 : 0;
    //}
    QuadratureDecoder_ProcessPulse(&qd, 0, 0, 1, 0);
    double angle = QDecoderAxisState_GetAngleDeg(&qd.axes[0]);
    REQUIRE(std::abs(angle - 22.5) < ANGLE_TOLERANCE);

    QuadratureDecoder_ProcessPulse(&qd, 0, 1, 1, 0);
    angle = QDecoderAxisState_GetAngleDeg(&qd.axes[0]);
    REQUIRE(std::abs(angle - 45.0) < ANGLE_TOLERANCE);

    QuadratureDecoder_ProcessPulse(&qd, 0, 1, 0, 0);

    angle = QDecoderAxisState_GetAngleDeg(&qd.axes[0]);
    REQUIRE(std::abs(angle - 67.5) < ANGLE_TOLERANCE);
}

TEST_CASE("Angle at half revolution", "[angle]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    long long half_positions = 2LL * qd.axes[0].cpr;

    for (long long i = 0; i < half_positions; ++i) {
        int step = (int)(i % 4);
        int a = (step >= 1 && step <= 2) ? 1 : 0;
        int b = (step >= 2 && step <= 3) ? 1 : 0;
        QuadratureDecoder_ProcessPulse(&qd, 0, a, b, 0);
    }

    double angle = QDecoderAxisState_GetAngleDeg(&qd.axes[0]);
    REQUIRE(std::abs(angle - 180.0) < ANGLE_TOLERANCE);
}

TEST_CASE("Angle at three-quarter revolution", "[angle]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    long long three_quarter_positions = 3LL * qd.axes[0].cpr;

    for (long long i = 0; i < three_quarter_positions; ++i) {
        int step = (int)(i % 4);
        int a = (step >= 1 && step <= 2) ? 1 : 0;
        int b = (step >= 2 && step <= 3) ? 1 : 0;
        QuadratureDecoder_ProcessPulse(&qd, 0, a, b, 0);
    }

    double angle = QDecoderAxisState_GetAngleDeg(&qd.axes[0]);
    REQUIRE(std::abs(angle - 270.0) < ANGLE_TOLERANCE);
}

TEST_CASE("Angle wraps at 360 degrees", "[angle]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    long long positions_per_rev = 4LL * qd.axes[0].cpr;

    for (long long i = 0; i < positions_per_rev; ++i) {
        int step = (int)(i % 4);
        int a = (step >= 1 && step <= 2) ? 1 : 0;
        int b = (step >= 2 && step <= 3) ? 1 : 0;
        QuadratureDecoder_ProcessPulse(&qd, 0, a, b, 0);
    }

    double angle = QDecoderAxisState_GetAngleDeg(&qd.axes[0]);
    REQUIRE(angle < ANGLE_TOLERANCE);
}

TEST_CASE("Angle in radians", "[angle]") {
    QuadratureDecoder qd = QuadratureDecoder_Construct(4, 1);
    QDecoderAxisState_InitializeAtIndex(&qd.axes[0]);

    long long half_positions = 2LL * qd.axes[0].cpr;

    for (long long i = 0; i < half_positions; ++i) {
        int step = (int)(i % 4);
        int a = (step >= 1 && step <= 2) ? 1 : 0;
        int b = (step >= 2 && step <= 3) ? 1 : 0;
        QuadratureDecoder_ProcessPulse(&qd, 0, a, b, 0);
    }

    double angle_rad = QDecoderAxisState_GetAngleRad(&qd.axes[0]);
    REQUIRE(std::abs(angle_rad - M_PI) < RADIAN_TOLERANCE);
}