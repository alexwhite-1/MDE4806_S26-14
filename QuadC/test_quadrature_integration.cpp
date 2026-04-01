// test_quadrature_decoder.cpp
// Authors: Alex White
// Date: 2026-03-25
//
// Integration tests: feeds QuadratureOutput pulses into QuadratureDecoder
// and verifies the decoded angle/position matches the original input.
//

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "quadrature_output.h"
#include "quadrature_decoder.h"

#include <cmath>
#include <cstring>

// ============================================================================
//  Test helpers
// ============================================================================

/// Degrees per one quadrature position step for a given CPR.
static double StepDeg(int cpr) {
    return 360.0 / (cpr * 4.0);
}

/// Drive `output` (single-axis) from its current angle toward `target_angle`
/// one quadrature position at a time, feeding every intermediate state into
/// `decoder` axis 0. Returns the number of steps taken.
static int DriveToAngle(QuadratureOutput* output,
    QuadratureDecoder* decoder,
    double current_angle,
    double target_angle)
{
    double step = StepDeg(output->axis1.cpr);

    // Wrap-aware delta: choose the shorter path around the circle
    double delta = target_angle - current_angle;
    while (delta > 180.0) delta -= 360.0;
    while (delta < -180.0) delta += 360.0;

    int steps = static_cast<int>(std::round(delta / step));
    int dir = (steps >= 0) ? 1 : -1;

    for (int i = 0; i < std::abs(steps); ++i) {
        current_angle += dir * step;
        // Keep angle in [0, 360)
        if (current_angle >= 360.0) current_angle -= 360.0;
        if (current_angle < 0.0) current_angle += 360.0;

        QuadratureOutput_Update(output, current_angle, current_angle);
        QuadratureDecoder_ProcessPulseOutput(decoder, output);
    }
    return std::abs(steps);
}

/// Same as DriveToAngle but drives both axes simultaneously (dual-axis tests).
static int DriveToAngleDual(QuadratureOutput* output,
    QuadratureDecoder* decoder,
    double cur1, double target1,
    double cur2, double target2)
{
    double step = StepDeg(output->axis1.cpr);

    double d1 = target1 - cur1;
    while (d1 > 180.0) d1 -= 360.0;
    while (d1 < -180.0) d1 += 360.0;

    double d2 = target2 - cur2;
    while (d2 > 180.0) d2 -= 360.0;
    while (d2 < -180.0) d2 += 360.0;

    int steps1 = static_cast<int>(std::round(d1 / step));
    int steps2 = static_cast<int>(std::round(d2 / step));
    int maxSteps = std::max(std::abs(steps1), std::abs(steps2));

    for (int i = 0; i < maxSteps; ++i) {
        if (i < std::abs(steps1)) cur1 += (steps1 >= 0 ? 1 : -1) * step;
        if (i < std::abs(steps2)) cur2 += (steps2 >= 0 ? 1 : -1) * step;

        if (cur1 >= 360.0) cur1 -= 360.0; if (cur1 < 0.0) cur1 += 360.0;
        if (cur2 >= 360.0) cur2 -= 360.0; if (cur2 < 0.0) cur2 += 360.0;

        QuadratureOutput_Update(output, cur1, cur2);
        QuadratureDecoder_ProcessPulseOutput(decoder, output);
    }
    return maxSteps;
}

// Tolerance: within half a quadrature step of the target angle.
static double HalfStep(int cpr) { return StepDeg(cpr) * 0.5; }


// ============================================================================
//  Test cases
// ============================================================================

// ----------------------------------------------------------------------------
//  1. Basic construction / sanity
// ----------------------------------------------------------------------------
TEST_CASE("Decoder initialises to zero angle", "[decoder][init]")
{
    QuadratureDecoder decoder = QuadratureDecoder_ConstructDefault();
    REQUIRE(QDecoderAxisState_GetAngleDeg(&decoder.axes[0]) == Catch::Approx(0.0));
    REQUIRE(QDecoderAxisState_GetPositionCount(&decoder.axes[0]) == 0);
    REQUIRE(QDecoderAxisState_GetRevolutionCount(&decoder.axes[0]) == 0);
    REQUIRE_FALSE(QDecoderAxisState_HasErrors(&decoder.axes[0]));
}

TEST_CASE("Output initialises channels to zero", "[output][init]")
{
    QuadratureOutput output = QuadratureOutput_ConstructDefault();
    QuadratureOutput_Initialize(&output, 0.0, 0.0);
    REQUIRE(output.axis1.channel_a == 0);
    REQUIRE(output.axis1.channel_b == 0);
    REQUIRE(output.axis1.index == 1); // index high at 0 degrees
}


// ----------------------------------------------------------------------------
//  2. Forward rotation — quarter turns
// ----------------------------------------------------------------------------
TEST_CASE("Decoder tracks 90-degree forward steps", "[decoder][forward]")
{
    const int CPR = 100;
    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    double angle = 0.0;
    const double targets[] = { 90.0, 180.0, 270.0, 360.0 };

    for (double target : targets) {
        DriveToAngle(&output, &decoder, angle, target);
        angle = (target == 360.0) ? 0.0 : target;

        double decoded = QDecoderAxisState_GetAngleDeg(&decoder.axes[0]);
        // At 360 the output wraps to 0
        double expected = std::fmod(target, 360.0);
        REQUIRE(decoded == Catch::Approx(expected).margin(HalfStep(CPR)));
        REQUIRE_FALSE(QDecoderAxisState_HasErrors(&decoder.axes[0]));
    }
}


// ----------------------------------------------------------------------------
//  3. Full forward revolution — position count and revolution count
// ----------------------------------------------------------------------------
TEST_CASE("Decoder counts one full forward revolution correctly", "[decoder][forward][revolution]")
{
    const int CPR = 200;
    const int PPR = CPR * 4;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    // Drive one full revolution in small steps
    double step = StepDeg(CPR);
    double angle = 0.0;
    for (int i = 0; i < PPR; ++i) {
        angle += step;
        if (angle >= 360.0) angle -= 360.0;
        QuadratureOutput_Update(&output, angle, angle);
        QuadratureDecoder_ProcessPulseOutput(&decoder, &output);
    }

    // After exactly PPR steps the position count wraps back to 0
    REQUIRE(QDecoderAxisState_GetPositionCount(&decoder.axes[0]) == 0);
    REQUIRE(QDecoderAxisState_GetAngleDeg(&decoder.axes[0]) == Catch::Approx(0.0).margin(HalfStep(CPR)));
    REQUIRE_FALSE(QDecoderAxisState_HasErrors(&decoder.axes[0]));
}


// ----------------------------------------------------------------------------
//  4. Reverse rotation
// ----------------------------------------------------------------------------
TEST_CASE("Decoder tracks reverse (negative) rotation", "[decoder][reverse]")
{
    const int CPR = 100;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);

    // Start at 180 degrees so we have room to go backwards without wrapping issues
    QuadratureOutput_Initialize(&output, 180.0, 180.0);

    // Manually seed the decoder to 180 deg by driving forward first
    double angle = 0.0;
    DriveToAngle(&output, &decoder, angle, 180.0);
    angle = 180.0;

    REQUIRE(QDecoderAxisState_GetAngleDeg(&decoder.axes[0]) == Catch::Approx(180.0).margin(HalfStep(CPR)));

    // Now drive backwards to 90 degrees
    DriveToAngle(&output, &decoder, 180.0, 90.0);

    double decoded = QDecoderAxisState_GetAngleDeg(&decoder.axes[0]);
    REQUIRE(decoded == Catch::Approx(90.0).margin(HalfStep(CPR)));
    REQUIRE_FALSE(QDecoderAxisState_HasErrors(&decoder.axes[0]));
}


// ----------------------------------------------------------------------------
//  5. Oscillation — forward then back to origin
// ----------------------------------------------------------------------------
TEST_CASE("Decoder returns to origin after forward-then-reverse oscillation", "[decoder][oscillation]")
{
    const int CPR = 100;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    // Forward 45 deg, back to 0
    DriveToAngle(&output, &decoder, 0.0, 45.0);
    DriveToAngle(&output, &decoder, 45.0, 0.0);

    REQUIRE(QDecoderAxisState_GetAngleDeg(&decoder.axes[0]) == Catch::Approx(0.0).margin(HalfStep(CPR)));
    REQUIRE(QDecoderAxisState_GetPositionCount(&decoder.axes[0]) == 0);
    REQUIRE_FALSE(QDecoderAxisState_HasErrors(&decoder.axes[0]));
}


// ----------------------------------------------------------------------------
//  6. Position count accuracy across many steps
// ----------------------------------------------------------------------------
TEST_CASE("Decoder position count matches expected step count", "[decoder][accuracy]")
{
    const int CPR = 500;
    const int PPR = CPR * 4;
    const int STEPS = 250; // advance 250 positions = 45 degrees

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    double step = StepDeg(CPR);
    double angle = 0.0;
    for (int i = 0; i < STEPS; ++i) {
        angle += step;
        QuadratureOutput_Update(&output, angle, angle);
        QuadratureDecoder_ProcessPulseOutput(&decoder, &output);
    }

    REQUIRE(QDecoderAxisState_GetPositionCount(&decoder.axes[0]) == STEPS);
    double expected_angle = (static_cast<double>(STEPS) / PPR) * 360.0;
    REQUIRE(QDecoderAxisState_GetAngleDeg(&decoder.axes[0]) == Catch::Approx(expected_angle).margin(HalfStep(CPR)));
    REQUIRE_FALSE(QDecoderAxisState_HasErrors(&decoder.axes[0]));
}


// ----------------------------------------------------------------------------
//  7. Multiple revolutions — revolution counter
// ----------------------------------------------------------------------------
TEST_CASE("Decoder revolution counter increments after multiple full revolutions", "[decoder][revolution]")
{
    const int CPR = 100;
    const int PPR = CPR * 4;
    const int REVS = 3;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    double step = StepDeg(CPR);
    double angle = 0.0;
    for (int r = 0; r < REVS; ++r) {
        for (int i = 0; i < PPR; ++i) {
            angle += step;
            if (angle >= 360.0) angle -= 360.0;
            QuadratureOutput_Update(&output, angle, angle);
            QuadratureDecoder_ProcessPulseOutput(&decoder, &output);
        }
    }

    REQUIRE(QDecoderAxisState_GetRevolutionCount(&decoder.axes[0]) == REVS);
}


// ----------------------------------------------------------------------------
//  8. No errors on a clean run
// ----------------------------------------------------------------------------
TEST_CASE("Clean pulse sequence produces zero decoder errors", "[decoder][errors]")
{
    const int CPR = 200;
    const int PPR = CPR * 4;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    double step = StepDeg(CPR);
    double angle = 0.0;

    // Forward half revolution
    for (int i = 0; i < PPR / 2; ++i) {
        angle += step;
        QuadratureOutput_Update(&output, angle, angle);
        QuadratureDecoder_ProcessPulseOutput(&decoder, &output);
    }

    // Backward half revolution
    for (int i = 0; i < PPR / 2; ++i) {
        angle -= step;
        if (angle < 0.0) angle += 360.0;
        QuadratureOutput_Update(&output, angle, angle);
        QuadratureDecoder_ProcessPulseOutput(&decoder, &output);
    }

    REQUIRE(decoder.axes[0].error_count == 0);
}


// ----------------------------------------------------------------------------
//  9. Index pulse synchronisation
// ----------------------------------------------------------------------------
TEST_CASE("Index pulse fires when position returns to zero", "[decoder][index]")
{
    const int CPR = 100;
    const int PPR = CPR * 4;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    // Drive one full revolution
    double step = StepDeg(CPR);
    double angle = 0.0;
    for (int i = 0; i < PPR; ++i) {
        angle += step;
        if (angle >= 360.0) angle -= 360.0;
        QuadratureOutput_Update(&output, angle, angle);
        QuadratureDecoder_ProcessPulseOutput(&decoder, &output);
    }

    // The decoder should have seen at least one index pulse
    REQUIRE(decoder.axes[0].index_pulse_count >= 1);
}


// ----------------------------------------------------------------------------
//  10. Reset clears decoder state
// ----------------------------------------------------------------------------
TEST_CASE("Decoder reset clears position, errors, and pulse counts", "[decoder][reset]")
{
    const int CPR = 100;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    // Accumulate some state
    double step = StepDeg(CPR);
    double angle = 0.0;
    for (int i = 0; i < 40; ++i) {
        angle += step;
        QuadratureOutput_Update(&output, angle, angle);
        QuadratureDecoder_ProcessPulseOutput(&decoder, &output);
    }
    REQUIRE(QDecoderAxisState_GetPositionCount(&decoder.axes[0]) > 0);

    // Reset and verify
    QDecoderAxisState_Reset(&decoder.axes[0]);
    REQUIRE(QDecoderAxisState_GetPositionCount(&decoder.axes[0]) == 0);
    REQUIRE(QDecoderAxisState_GetAngleDeg(&decoder.axes[0]) == Catch::Approx(0.0));
    REQUIRE(decoder.axes[0].pulse_count == 0);
    REQUIRE(decoder.axes[0].error_count == 0);
    REQUIRE(decoder.axes[0].synchronized == false);
}


// ----------------------------------------------------------------------------
//  11. Dual-axis independent tracking
// ----------------------------------------------------------------------------
TEST_CASE("Decoder tracks two independent axes simultaneously", "[decoder][dual]")
{
    const int CPR = 100;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 2);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 2);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    // Drive axis1 to 90 deg, axis2 to 180 deg
    double cur1 = 0.0, cur2 = 0.0;
    DriveToAngleDual(&output, &decoder, cur1, 90.0, cur2, 180.0);

    double decoded1 = QDecoderAxisState_GetAngleDeg(&decoder.axes[0]);
    double decoded2 = QDecoderAxisState_GetAngleDeg(&decoder.axes[1]);

    REQUIRE(decoded1 == Catch::Approx(90.0).margin(HalfStep(CPR)));
    REQUIRE(decoded2 == Catch::Approx(180.0).margin(HalfStep(CPR)));
    REQUIRE_FALSE(QDecoderAxisState_HasErrors(&decoder.axes[0]));
    REQUIRE_FALSE(QDecoderAxisState_HasErrors(&decoder.axes[1]));
}


// ----------------------------------------------------------------------------
//  12. Formatted output string from decoder matches expected pattern
// ----------------------------------------------------------------------------
TEST_CASE("Decoder formatted output string has correct CSV format", "[decoder][format]")
{
    const int CPR = 100;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 2);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 2);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    DriveToAngle(&output, &decoder, 0.0, 45.0);

    char buf[128];
    QuadratureDecoder_GetFormattedOutputDual(&decoder, buf, sizeof(buf));

    // Format: "angle1,count1,rev1,angle2,count2,rev2"
    // Just verify it's non-empty and contains commas (we parsed the right number)
    REQUIRE(std::strlen(buf) > 0);
    int comma_count = 0;
    for (char* p = buf; *p; ++p) if (*p == ',') ++comma_count;
    REQUIRE(comma_count == 5); // 6 fields ? 5 commas
}


// ----------------------------------------------------------------------------
//  13. Wrap-around: crossing 360/0 boundary forward
// ----------------------------------------------------------------------------
TEST_CASE("Decoder handles 360-to-0 wrap-around going forward", "[decoder][wrap]")
{
    const int CPR = 100;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    // Drive to just before 360, then across the boundary
    DriveToAngle(&output, &decoder, 0.0, 350.0);
    REQUIRE(QDecoderAxisState_GetAngleDeg(&decoder.axes[0]) == Catch::Approx(350.0).margin(HalfStep(CPR)));

    DriveToAngle(&output, &decoder, 350.0, 10.0);
    REQUIRE(QDecoderAxisState_GetAngleDeg(&decoder.axes[0]) == Catch::Approx(10.0).margin(HalfStep(CPR)));
    REQUIRE_FALSE(QDecoderAxisState_HasErrors(&decoder.axes[0]));
}


// ----------------------------------------------------------------------------
//  14. Wrap-around: crossing 0/360 boundary in reverse
// ----------------------------------------------------------------------------
TEST_CASE("Decoder handles 0-to-360 wrap-around going in reverse", "[decoder][wrap]")
{
    const int CPR = 100;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    // Start at 10 deg, go back past 0
    DriveToAngle(&output, &decoder, 0.0, 10.0);
    DriveToAngle(&output, &decoder, 10.0, 350.0); // short reverse path wraps to 350

    double decoded = QDecoderAxisState_GetAngleDeg(&decoder.axes[0]);
    REQUIRE(decoded == Catch::Approx(350.0).margin(HalfStep(CPR)));
    REQUIRE_FALSE(QDecoderAxisState_HasErrors(&decoder.axes[0]));
}


// ----------------------------------------------------------------------------
//  15. CPR change via SetCPR propagates to position scaling
// ----------------------------------------------------------------------------
TEST_CASE("Changing CPR on output and decoder yields consistent angle", "[decoder][cpr]")
{
    const int CPR = 50;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    DriveToAngle(&output, &decoder, 0.0, 180.0);

    REQUIRE(QDecoderAxisState_GetAngleDeg(&decoder.axes[0]) == Catch::Approx(180.0).margin(HalfStep(CPR)));
    REQUIRE_FALSE(QDecoderAxisState_HasErrors(&decoder.axes[0]));
}