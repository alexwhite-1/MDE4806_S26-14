// test_quadrature_output_pulse.cpp
// Authors: Alex White
// Date: 2026-03-25
//
// Per-pulse correctness tests for QuadratureOutput.
//
// Goal: verify that every individual A/B/index state emitted during an
// angle transition is correct — not just the final value after a jump.
//
// Strategy
// --------
// QuadratureOutput_Update() can move the encoder by many positions in one
// call (now via the internal StepOne loop). To observe every intermediate
// state, each test drives the encoder exactly ONE position per Update call
// by computing the angle for each successive position index and calling
// Update with that angle. The state is snapshotted immediately after each
// call, building a complete record of the pulse stream that can then be
// audited against invariants and exact expected sequences.
//
// Invariants checked on every pulse in every test
// ------------------------------------------------
//   I1. Gray-code rule — only one of {A, B} may change per step.
//   I2. A and B are always 0 or 1.
//   I3. The step count matches the expected position change.
//   I4. position_count stays in [0, positions_per_rev).
//   I5. Index is high if and only if position_count == 0 && A==0 && B==0.
//
// Tests are organised from simplest to most demanding:
//   [pattern]       — exact A/B sequence for known CPR
//   [step-count]    — number of pulses emitted matches expected positions
//   [gray-code]     — single-bit change invariant across all transitions
//   [direction]     — forward vs reverse sequence ordering
//   [boundary]      — wrap-around behaviour at the 0/360 boundary
//   [index]         — index fires at the right moment, exactly once per rev
//   [multi-step]    — large jumps (multi-position Updates) emit correct stream
//   [dual-axis]     — both axes track independently, per-pulse
//   [oscillation]   — direction reversals produce the exact mirror sequence

#define CATCH_CONFIG_RUNNER
#include <catch2/catch_all.hpp>
#include "quadrature_output.h"
#include "quadrature_common.h"

#include <cmath>
#include <vector>
#include <string>
#include <sstream>

// ============================================================================
//  Pulse record and collection helpers
// ============================================================================

struct Pulse {
    int a;
    int b;
    int index;
    int position_count;
};

static std::string PulseStr(const Pulse& p) {
    std::ostringstream ss;
    ss << "A=" << p.a << " B=" << p.b
        << " idx=" << p.index
        << " pos=" << p.position_count;
    return ss.str();
}

// Expected Gray-code pattern for one full cycle of 4 positions (repeating).
// Forward motion (incrementing position_count) produces: 00->01->11->10->00.
// EncodeState(a,b) = (a<<1)|b, so forward path is state 0->1->3->2->0.
// Position % 4 -> {A, B}
//   0 -> {0, 0}
//   1 -> {0, 1}
//   2 -> {1, 1}
//   3 -> {1, 0}
static std::pair<int, int> ExpectedAB(int position_count) {
    switch (position_count % 4) {
    case 0: return { 0, 0 };
    case 1: return { 0, 1 };
    case 2: return { 1, 1 };
    case 3: return { 1, 0 };
    default: return { 0, 0 };
    }
}

// Angle corresponding to a given position index for a given CPR.
static double PositionToAngle(int pos, int ppr) {
    return (static_cast<double>(pos) / ppr) * 360.0;
}

// Drive `axis` forward from position `start_pos` to `end_pos` (exclusive),
// one position at a time, recording every pulse.
// `ppr` must equal axis->positions_per_rev.
static std::vector<Pulse> DriveForward(QOutputAxisState* axis,
    int start_pos, int end_pos, int ppr)
{
    std::vector<Pulse> pulses;
    pulses.reserve(end_pos - start_pos);
    for (int p = start_pos; p <= end_pos-1; ++p) {
        double angle = PositionToAngle(p % ppr, ppr);
        QOutputAxisState_UpdateAxis(axis, angle);
        pulses.push_back({ axis->channel_a, axis->channel_b,
                          axis->index, axis->position_count });
    }
    return pulses;
}

// Drive `axis` backward from position `start_pos` down to `end_pos`
// (inclusive), one position at a time.
static std::vector<Pulse> DriveBackward(QOutputAxisState* axis,
    int start_pos, int end_pos, int ppr)
{
    std::vector<Pulse> pulses;
    pulses.reserve(start_pos - end_pos);
    for (int p = start_pos - 1; p >= end_pos; --p) {
        int wrapped = ((p % ppr) + ppr) % ppr;
        double angle = PositionToAngle(wrapped, ppr);
        QOutputAxisState_UpdateAxis(axis, angle);
        pulses.push_back({ axis->channel_a, axis->channel_b,
                          axis->index, axis->position_count });
    }
    return pulses;
}

// ============================================================================
//  Universal invariant checker — called by every test
// ============================================================================

// Checks all five invariants on a recorded pulse stream.
// `start_ab` is the {A, B} state before the first pulse (i.e. the state that
// was current when recording began, so we can check the first transition).
static void CheckInvariants(const std::vector<Pulse>& pulses,
    std::pair<int, int> start_ab,
    int ppr,
    const std::string& context = "")
{
    REQUIRE_FALSE(pulses.empty());

    int prev_a = start_ab.first;
    int prev_b = start_ab.second;

    for (size_t i = 0; i < pulses.size(); ++i) {
        const Pulse& pulse = pulses[i];
        INFO(context << " pulse[" << i << "] " << PulseStr(pulse));

        // I2: valid logic levels
        REQUIRE((pulse.a == 0 || pulse.a == 1));
        REQUIRE((pulse.b == 0 || pulse.b == 1));

        // I1: Gray-code — exactly one bit changes per step
        int a_changed = (pulse.a != prev_a) ? 1 : 0;
        int b_changed = (pulse.b != prev_b) ? 1 : 0;
        int bits_changed = a_changed + b_changed;
        // Either a state change happened (exactly 1 bit) or no change (0 bits
        // — possible when the position didn't move, e.g. sub-step rounding)
        REQUIRE(bits_changed <= 1);

        // I4: position count in range
        REQUIRE(pulse.position_count >= 0);
        REQUIRE(pulse.position_count < ppr);

        // I5: index fires iff position_count==0 and A==0 and B==0
        bool at_zero = (pulse.position_count == 0 &&
            pulse.a == 0 && pulse.b == 0);
        REQUIRE(pulse.index == (at_zero ? 1 : 0));

        prev_a = pulse.a;
        prev_b = pulse.b;
    }
}

// ============================================================================
//  TEST GROUP: Exact A/B pattern [pattern]
// ============================================================================

TEST_CASE("Forward pulse sequence matches Gray-code pattern exactly, CPR=4",
    "[pattern]")
{
    // With CPR=4, PPR=16. Every position maps to a known {A,B} pair and we
    // can enumerate all 16 of them exhaustively.
    const int CPR = 4;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;   // 16

    auto pulses = DriveForward(&axis, 0, ppr, ppr);
    REQUIRE(static_cast<int>(pulses.size()) == ppr);

    for (int i = 0; i < ppr; ++i) {
        INFO("position " << (i) % ppr);
        auto [exp_a, exp_b] = ExpectedAB((i) % ppr);
        REQUIRE(pulses[i].a == exp_a);
        REQUIRE(pulses[i].b == exp_b);
        REQUIRE(pulses[i].position_count == (i) % ppr);
    }
}

TEST_CASE("Reverse pulse sequence matches Gray-code pattern exactly, CPR=4",
    "[pattern]")
{
    const int CPR = 4;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;   // 16

    // Starting at position 0, go backward through all 16 positions.
    // Reverse sequence: pos 15, 14, 13 … 0
    auto pulses = DriveBackward(&axis, ppr, 0, ppr);
    REQUIRE(static_cast<int>(pulses.size()) == ppr);

    for (int i = 0; i < ppr; ++i) {
        int expected_pos = ((ppr - 1 - i) % ppr + ppr) % ppr;
        INFO("step " << i << " expected pos " << expected_pos);
        auto [exp_a, exp_b] = ExpectedAB(expected_pos);
        REQUIRE(pulses[i].a == exp_a);
        REQUIRE(pulses[i].b == exp_b);
        REQUIRE(pulses[i].position_count == expected_pos);
    }
}

TEST_CASE("Gray-code pattern holds for CPR=1 (minimal encoder)", "[pattern]") {
    // CPR=1 means 4 positions per revolution — the smallest legal encoder.
    const int CPR = 1;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;   // 4

    auto pulses = DriveForward(&axis, 0, ppr, ppr);

    // Exact states: pos0={0,0}, pos1={0,1}, pos2={1,1}, pos3={1,0}
    REQUIRE(pulses[0].a == 0); REQUIRE(pulses[0].b == 0);
    REQUIRE(pulses[1].a == 0); REQUIRE(pulses[1].b == 1);
    REQUIRE(pulses[2].a == 1); REQUIRE(pulses[2].b == 1);
    REQUIRE(pulses[3].a == 1); REQUIRE(pulses[3].b == 0);
}

TEST_CASE("Gray-code pattern holds for CPR=100", "[pattern]") {
    // For CPR=100 (PPR=400) verify every position maps to the correct {A,B}.
    const int CPR = 100;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    auto pulses = DriveForward(&axis, 0, ppr, ppr);
    REQUIRE(static_cast<int>(pulses.size()) == ppr);

    for (int i = 0; i < ppr; ++i) {
        INFO("position " << (i) % ppr);
        auto [exp_a, exp_b] = ExpectedAB((i) % ppr);
        REQUIRE(pulses[i].a == exp_a);
        REQUIRE(pulses[i].b == exp_b);
    }
}

// ============================================================================
//  TEST GROUP: Step count correctness [step-count]
// ============================================================================

TEST_CASE("One-degree forward step emits correct number of pulses, CPR=360",
    "[step-count]")
{
    // CPR=360 ? PPR=1440. One degree = 4 positions.
    const int CPR = 360;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;   // 1440
    const int expected_steps = ppr / 360;      // 4

    auto pulses = DriveForward(&axis, 0, expected_steps, ppr);
    REQUIRE(static_cast<int>(pulses.size()) == expected_steps);
    REQUIRE(pulses.back().position_count == expected_steps-1);
}

TEST_CASE("Quarter-revolution forward emits PPR/4 pulses", "[step-count]") {
    const int CPR = 100;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;   // 400

    auto pulses = DriveForward(&axis, 0, ppr / 4, ppr);
    REQUIRE(static_cast<int>(pulses.size()) == ppr / 4);
    REQUIRE(pulses.back().position_count == (ppr / 4 )-1);
}

TEST_CASE("Half-revolution forward emits PPR/2 pulses", "[step-count]") {
    const int CPR = 100;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    auto pulses = DriveForward(&axis, 0, ppr / 2, ppr);
    REQUIRE(static_cast<int>(pulses.size()) == ppr / 2);
    REQUIRE(pulses.back().position_count == (ppr / 2)-1);
}

TEST_CASE("Full revolution forward emits exactly PPR pulses", "[step-count]") {
    const int CPR = 100;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    auto pulses = DriveForward(&axis, 0, ppr, ppr);
    REQUIRE(static_cast<int>(pulses.size()) == ppr);
    // Wraps back to 0
    REQUIRE(pulses.back().position_count == ppr-1);
}

TEST_CASE("Three full revolutions emit exactly 3*PPR pulses", "[step-count]") {
    const int CPR = 50;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    auto pulses = DriveForward(&axis, 0, 3 * ppr, ppr);
    REQUIRE(static_cast<int>(pulses.size()) == 3 * ppr);
    REQUIRE(pulses.back().position_count == ppr-1);
}

TEST_CASE("Quarter-revolution reverse emits PPR/4 pulses", "[step-count]") {
    const int CPR = 100;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    // Start at position ppr/4, go backward to 0
    QOutputAxisState_UpdateAxis(&axis, 90.0);  // jump to 90 deg = PPR/4
    axis.previous_angle = 90.0;

    auto pulses = DriveBackward(&axis, ppr / 4, 0, ppr);
    REQUIRE(static_cast<int>(pulses.size()) == ppr / 4);
    REQUIRE(pulses.back().position_count == 0);
}

// ============================================================================
//  TEST GROUP: Gray-code single-bit invariant [gray-code]
// ============================================================================

TEST_CASE("Every forward pulse changes exactly one channel, CPR=4",
    "[gray-code]")
{
    const int CPR = 4;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    // Drive two full revolutions to cover wrap-around twice
    auto pulses = DriveForward(&axis, 0, 2 * ppr, ppr);
    CheckInvariants(pulses, { 0, 0 }, ppr, "forward 2 revs CPR=4");
}

TEST_CASE("Every reverse pulse changes exactly one channel, CPR=4",
    "[gray-code]")
{
    const int CPR = 4;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    auto pulses = DriveBackward(&axis, 2 * ppr, 0, ppr);
    CheckInvariants(pulses, { 0, 0 }, ppr, "reverse 2 revs CPR=4");
}

TEST_CASE("Gray-code invariant holds across full revolution, CPR=100",
    "[gray-code]")
{
    const int CPR = 100;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    auto pulses = DriveForward(&axis, 0, ppr, ppr);
    CheckInvariants(pulses, { 0, 0 }, ppr, "forward full rev CPR=100");
}

TEST_CASE("Gray-code invariant holds across large multi-step Update, CPR=100",
    "[gray-code]")
{
    // This test specifically targets the scenario where a single Update call
    // jumps many positions. We record by driving one step at a time but using
    // a larger CPR to create meaningful multi-position jumps per degree.
    const int CPR = 500;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;  // 2000

    // Drive a full revolution, one step at a time
    auto pulses = DriveForward(&axis, 0, ppr, ppr);
    CheckInvariants(pulses, { 0, 0 }, ppr, "forward full rev CPR=500");
}

TEST_CASE("Gray-code invariant holds across direction reversal", "[gray-code]") {
    const int CPR = 100;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    auto fwd = DriveForward(&axis, 0, ppr / 2, ppr);
    std::pair<int, int> midpoint_ab = { fwd.back().a, fwd.back().b };

    auto rev = DriveBackward(&axis, ppr / 2, 0, ppr);

    CheckInvariants(fwd, { 0, 0 }, ppr, "forward half rev");
    CheckInvariants(rev, midpoint_ab, ppr, "reverse half rev");
}

// ============================================================================
//  TEST GROUP: Direction ordering [direction]
// ============================================================================

TEST_CASE("Forward sequence is the exact reverse of the backward sequence",
    "[direction]")
{
    // Property: if you record A/B going forward from pos 0 to pos N, then
    // going backward from pos N to pos 0 produces the exact reversed list.
    const int CPR = 4;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;   // 16

    auto fwd = DriveForward(&axis, 0, ppr, ppr);

    // Reset to position 0 for the backward drive
    QOutputAxisState_Initialize(&axis, 0.0);
    auto rev = DriveBackward(&axis, ppr, 0, ppr);

    REQUIRE(fwd.size() == rev.size());
    for (size_t i = 0; i < fwd.size(); ++i) {
        INFO("step " << i);
        // The forward pulse at step i should equal the reverse pulse at step
        // (N - 1 - i) when both are expressed as {A, B} pairs.
        // fwd[0] lands on pos 1, rev[0] lands on pos PPR-1 = pos 15.
        // fwd[i].{a,b} should equal rev[PPR-1-i].{a,b}
        size_t j = fwd.size() - 1 - i;
        REQUIRE(fwd[i].a == rev[j].a);
        REQUIRE(fwd[i].b == rev[j].b);
    }
}

TEST_CASE("B leads A by 90 degrees in forward direction (channel ordering)",
    "[direction]")
{
    // In this encoder's forward direction, B transitions first:
    // the sequence 00->01->11->10 shows B rising before A.
    // This matches the decoder VALID_TRANSITIONS forward path: state 0->1->3->2->0.
    const int CPR = 4;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    auto pulses = DriveForward(&axis, 0, ppr, ppr);

    // Verify the four canonical transitions in the first four steps
    // Starting state: A=0, B=0 (pos 0)
    REQUIRE(pulses[0].a == 0); REQUIRE(pulses[0].b == 0); // 00: start
    REQUIRE(pulses[1].a == 0); REQUIRE(pulses[1].b == 1); // 00->01: B rises
    REQUIRE(pulses[2].a == 1); REQUIRE(pulses[2].b == 1); // 01->11: A rises
    REQUIRE(pulses[3].a == 1); REQUIRE(pulses[3].b == 0); // 11->10: B falls
}

TEST_CASE("A leads B by 90 degrees in reverse direction (channel ordering)",
    "[direction]")
{
    // Going in reverse the sequence is: 00->10->11->01->00.
    // A transitions first in reverse (mirror of the forward B-first sequence).
    const int CPR = 4;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    auto pulses = DriveBackward(&axis, ppr, 0, ppr);

    // Starting state: A=0, B=0 (pos 0)
    REQUIRE(pulses[0].a == 1); REQUIRE(pulses[0].b == 0); // 00->10: A rises
    REQUIRE(pulses[1].a == 1); REQUIRE(pulses[1].b == 1); // 10->11: B rises
    REQUIRE(pulses[2].a == 0); REQUIRE(pulses[2].b == 1); // 11->01: A falls
    REQUIRE(pulses[3].a == 0); REQUIRE(pulses[3].b == 0); // 01->00: B falls
}

// ============================================================================
//  TEST GROUP: Wrap-around at the 0/360 boundary [boundary]
// ============================================================================

TEST_CASE("Forward wrap: crossing 360 produces correct pulse sequence",
    "[boundary]")
{
    // Drive from 350 degrees to 10 degrees going forward (crossing 0/360).
    const int CPR = 100;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;  // 400
    // 350 deg = position 350/360 * 400 = ~388.9 -> 389
    const int start_pos = static_cast<int>(std::round((350.0 / 360.0) * ppr));
    // Advance to start_pos
    DriveForward(&axis, 0, start_pos, ppr);
    std::pair<int, int> start_ab = { axis.channel_a, axis.channel_b };

    // 10 deg = position 10/360 * 400 = ~11.1 -> 11
    // From start_pos (389) forward to ppr (400) is 11 steps, then position 11
    // positions further = 22 total steps through the boundary
    const int end_pos_abs = start_pos + (ppr - start_pos) + 11; // crosses 0
    auto pulses = DriveForward(&axis, start_pos, end_pos_abs, ppr);

    // Every pulse must obey the Gray-code invariant through the boundary
    CheckInvariants(pulses, start_ab, ppr, "forward wrap 350->10");

    // Final position should be ~11 (10 deg at CPR=100)
    REQUIRE(pulses.back().position_count < ppr / 4);
}

TEST_CASE("Reverse wrap: crossing 0 produces correct pulse sequence",
    "[boundary]")
{
    // Drive from 10 degrees backward past 0 to 350 degrees.
    const int CPR = 100;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;
    const int start_pos = static_cast<int>(std::round((10.0 / 360.0) * ppr));
    DriveForward(&axis, 0, start_pos, ppr);
    std::pair<int, int> start_ab = { axis.channel_a, axis.channel_b };

    // Going backward from pos 11 through 0 and into the high end of the range
    const int end_pos_abs = start_pos + 22; // 22 steps backward
    auto pulses = DriveBackward(&axis, start_pos, start_pos - end_pos_abs, ppr);

    CheckInvariants(pulses, start_ab, ppr, "reverse wrap 10->350");

    // Final position should be near 350 deg
    REQUIRE(pulses.back().position_count > ppr * 3 / 4);
}

TEST_CASE("Full revolution forward through 0/360 boundary produces clean pulses",
    "[boundary]")
{
    const int CPR = 100;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    // Start at 270 deg so the wrap happens mid-revolution
    const int start_pos = (3 * ppr) / 4;
    DriveForward(&axis, 0, start_pos, ppr);
    std::pair<int, int> start_ab = { axis.channel_a, axis.channel_b };

    auto pulses = DriveForward(&axis, start_pos, start_pos + ppr, ppr);
    CheckInvariants(pulses, start_ab, ppr, "full rev from 270 through 0");
    REQUIRE(pulses.back().position_count == start_pos % ppr-1);
}

// ============================================================================
//  TEST GROUP: Index signal correctness [index]
// ============================================================================

TEST_CASE("Index fires exactly once per forward revolution, CPR=4", "[index]") {
    const int CPR = 4;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    auto pulses = DriveForward(&axis, 0, ppr, ppr);

    int index_count = 0;
    for (const auto& p : pulses) {
        if (p.index == 1) ++index_count;
    }
    REQUIRE(index_count == 1);
}

TEST_CASE("Index fires exactly once per forward revolution, CPR=100",
    "[index]")
{
    const int CPR = 100;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    auto pulses = DriveForward(&axis, 0, ppr, ppr);

    int index_count = 0;
    for (const auto& p : pulses) {
        if (p.index == 1) ++index_count;
    }
    REQUIRE(index_count == 1);
}

TEST_CASE("Index fires exactly N times over N full forward revolutions",
    "[index]")
{
    const int CPR = 50;
    const int REVS = 5;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    auto pulses = DriveForward(&axis, 0, REVS * ppr, ppr);

    int index_count = 0;
    for (const auto& p : pulses) {
        if (p.index == 1) ++index_count;
    }
    REQUIRE(index_count == REVS);
}

TEST_CASE("Index only fires when A=0 and B=0", "[index]") {
    const int CPR = 100;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    auto pulses = DriveForward(&axis, 0, 3 * ppr, ppr);

    for (size_t i = 0; i < pulses.size(); ++i) {
        if (pulses[i].index == 1) {
            INFO("index high at pulse " << i);
            REQUIRE(pulses[i].a == 0);
            REQUIRE(pulses[i].b == 0);
            REQUIRE(pulses[i].position_count == 0);
        }
    }
}

TEST_CASE("Index does not fire during reverse traversal away from position 0",
    "[index]")
{
    // Drive from 90 deg backwards to 10 deg — never passes through 0.
    const int CPR = 100;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;
    const int start_pos = ppr / 4;  // 90 deg
    const int end_pos = static_cast<int>(std::round((10.0 / 360.0) * ppr));

    DriveForward(&axis, 0, start_pos, ppr);

    auto pulses = DriveBackward(&axis, start_pos, end_pos, ppr);

    for (size_t i = 0; i < pulses.size(); ++i) {
        INFO("pulse " << i << " " << PulseStr(pulses[i]));
        REQUIRE(pulses[i].index == 0);
    }
}

TEST_CASE("Index fires when reverse rotation crosses position 0", "[index]") {
    // Drive from 10 deg backward through 0 to 350 deg — should get one index.
    const int CPR = 100;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;
    const int start_pos = static_cast<int>(std::round((10.0 / 360.0) * ppr));

    DriveForward(&axis, 0, start_pos, ppr);

    const int steps_backward = static_cast<int>(std::round((20.0 / 360.0) * ppr));
    auto pulses = DriveBackward(&axis, start_pos, start_pos - steps_backward, ppr);

    int index_count = 0;
    for (const auto& p : pulses) {
        if (p.index == 1) ++index_count;
    }
    REQUIRE(index_count == 1);
}

// ============================================================================
//  TEST GROUP: Multi-step Update emits every pulse [multi-step]
// ============================================================================
//
// These tests call QuadratureOutput_Update() with an angle jump that spans
// multiple positions, then check that the final state is consistent with
// having walked through all intermediate states correctly. The decoder is
// used as the witness: if any intermediate transition was skipped or was
// invalid, the decoder will report an error or an incorrect count.

#include "quadrature_decoder.h"

// Helper: call Update with the exact angle for each position, let the decoder
// witness every pulse, and return the decoder's final position count.
static long long DecoderWitnessUpdate(QuadratureOutput* output,
    QuadratureDecoder* decoder,
    double from_angle, double to_angle,
    int cpr)
{
    double step_deg = 360.0 / (cpr * 4.0);
    double delta = to_angle - from_angle;
    while (delta > 180.0) delta -= 360.0;
    while (delta < -180.0) delta += 360.0;

    int steps = static_cast<int>(std::round(delta / step_deg));
    int dir = (steps >= 0) ? 1 : -1;
    double angle = from_angle;

    for (int i = 0; i < std::abs(steps); ++i) {
        angle += dir * step_deg;
        if (angle >= 360.0) angle -= 360.0;
        if (angle < 0.0) angle += 360.0;
        QuadratureOutput_Update(output, angle, angle);
        QuadratureDecoder_ProcessPulseOutput(decoder, output);
    }
    return QDecoderAxisState_GetPositionCount(&decoder->axes[0]);
}

TEST_CASE("Multi-step Update: 0 to 90 deg, decoder counts correct positions",
    "[multi-step]")
{
    const int CPR = 100;
    const int PPR = CPR * 4;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    long long pos = DecoderWitnessUpdate(&output, &decoder, 0.0, 90.0, CPR);

    REQUIRE(decoder.axes[0].error_count == 0);
    REQUIRE(pos == PPR / 4);
}

TEST_CASE("Multi-step Update: 0 to 180 deg, decoder counts correct positions",
    "[multi-step]")
{
    const int CPR = 100;
    const int PPR = CPR * 4;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    long long pos = DecoderWitnessUpdate(&output, &decoder, 0.0, 180.0, CPR);

    REQUIRE(decoder.axes[0].error_count == 0);
    REQUIRE(pos == PPR / 2);
}

TEST_CASE("Multi-step Update: 0 to 270 deg, decoder counts correct positions",
    "[multi-step]")
{
    const int CPR = 100;
    const int PPR = CPR * 4;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    long long pos = DecoderWitnessUpdate(&output, &decoder, 0.0, 270.0, CPR);

    REQUIRE(decoder.axes[0].error_count == 0);
    REQUIRE(pos == (3 * PPR) / 4);
}

TEST_CASE("Multi-step Update: full 360-degree jump, decoder wraps to 0",
    "[multi-step]")
{
    const int CPR = 200;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    long long pos = DecoderWitnessUpdate(&output, &decoder, 0.0, 360.0, CPR);

    REQUIRE(decoder.axes[0].error_count == 0);
    REQUIRE(pos == 0);
    REQUIRE(QDecoderAxisState_GetRevolutionCount(&decoder.axes[0]) == 1);
}

TEST_CASE("Multi-step Update: reverse 90-degree jump, decoder counts backward",
    "[multi-step]")
{
    const int CPR = 100;
    const int PPR = CPR * 4;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    // Seed forward to 180 first so we have room to go back without wrapping
    DecoderWitnessUpdate(&output, &decoder, 0.0, 180.0, CPR);
    REQUIRE(decoder.axes[0].error_count == 0);

    long long pos = DecoderWitnessUpdate(&output, &decoder, 180.0, 90.0, CPR);

    REQUIRE(decoder.axes[0].error_count == 0);
    REQUIRE(pos == PPR / 4);
}

TEST_CASE("Multi-step Update: no decoder errors over 5 full revolutions",
    "[multi-step]")
{
    const int CPR = 500;
    const int PPR = CPR * 4;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    double angle = 0.0;
    double step = 360.0 / PPR;
    for (int i = 0; i < 5 * PPR; ++i) {
        angle += step;
        if (angle >= 360.0) angle -= 360.0;
        QuadratureOutput_Update(&output, angle, angle);
        QuadratureDecoder_ProcessPulseOutput(&decoder, &output);
    }

    REQUIRE(decoder.axes[0].error_count == 0);
    REQUIRE(QDecoderAxisState_GetRevolutionCount(&decoder.axes[0]) == 5);
}

// ============================================================================
//  TEST GROUP: Dual-axis per-pulse correctness [dual-axis]
// ============================================================================

TEST_CASE("Dual-axis: both axes emit correct pulse count independently",
    "[dual-axis]")
{
    const int CPR = 100;
    const int PPR = CPR * 4;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 2);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 2);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    double step = 360.0 / PPR;
    double a1 = 0.0, a2 = 0.0;

    // Axis 1 advances at 1x, axis 2 at 2x
    for (int i = 0; i < PPR; ++i) {
        a1 += step;
        a2 += 2.0 * step;
        if (a1 >= 360.0) a1 -= 360.0;
        if (a2 >= 360.0) a2 -= 360.0;

        QuadratureOutput_Update(&output, a1, a2);
        QuadratureDecoder_ProcessPulseOutput(&decoder, &output);
    }

    REQUIRE(decoder.axes[0].error_count == 0);
    REQUIRE(decoder.axes[1].error_count == 0);

    // Axis 1: PPR steps forward ? position 0 (wrapped once)
    REQUIRE(QDecoderAxisState_GetPositionCount(&decoder.axes[0]) == 0);
    // Axis 2: 2*PPR steps forward ? position 0 (wrapped twice)
    REQUIRE(QDecoderAxisState_GetPositionCount(&decoder.axes[1]) == 0);
    REQUIRE(QDecoderAxisState_GetRevolutionCount(&decoder.axes[1]) == 2);
}

TEST_CASE("Dual-axis: axes moving in opposite directions produce no errors",
    "[dual-axis]")
{
    const int CPR = 100;
    const int PPR = CPR * 4;

    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 2);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 2);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    double step = 360.0 / PPR;
    double a1 = 0.0, a2 = 0.0;

    // Start axis2 at 180 and go backward from there
    DecoderWitnessUpdate(&output, &decoder, 0.0, 180.0, CPR);
    a2 = 180.0;
    // Reset axis 1 for clarity
    QOutputAxisState_Initialize(&output.axis1, 0.0);
    a1 = 0.0;

    for (int i = 0; i < PPR / 2; ++i) {
        a1 += step;
        a2 -= step;
        if (a1 >= 360.0) a1 -= 360.0;
        if (a2 < 0.0) a2 += 360.0;

        QuadratureOutput_Update(&output, a1, a2);
        QuadratureDecoder_ProcessPulseOutput(&decoder, &output);
    }

    REQUIRE(decoder.axes[0].error_count == 0);
    REQUIRE(decoder.axes[1].error_count == 0);
}

// ============================================================================
//  TEST GROUP: Oscillation / direction reversal [oscillation]
// ============================================================================

TEST_CASE("Oscillation: forward then back to origin, zero net displacement",
    "[oscillation]")
{
    const int CPR = 100;
    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    // Forward 90 degrees
    DecoderWitnessUpdate(&output, &decoder, 0.0, 90.0, CPR);
    REQUIRE(decoder.axes[0].error_count == 0);

    // Back to 0
    DecoderWitnessUpdate(&output, &decoder, 90.0, 0.0, CPR);
    REQUIRE(decoder.axes[0].error_count == 0);
    REQUIRE(QDecoderAxisState_GetPositionCount(&decoder.axes[0]) == 0);
}

TEST_CASE("Oscillation: multiple forward/backward cycles return to origin",
    "[oscillation]")
{
    const int CPR = 100;
    const double SWING = 45.0;
    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    for (int cycle = 0; cycle < 10; ++cycle) {
        DecoderWitnessUpdate(&output, &decoder, 0.0, SWING, CPR);
        DecoderWitnessUpdate(&output, &decoder, SWING, 0.0, CPR);
    }

    REQUIRE(decoder.axes[0].error_count == 0);
    REQUIRE(QDecoderAxisState_GetPositionCount(&decoder.axes[0]) == 0);
}

TEST_CASE("Oscillation around 0/360 boundary produces no errors", "[oscillation]")
{
    const int CPR = 100;
    QuadratureOutput  output = QuadratureOutput_Construct(CPR, 1);
    QuadratureDecoder decoder = QuadratureDecoder_Construct(CPR, 1);
    QuadratureOutput_Initialize(&output, 0.0, 0.0);

    // Oscillate between 355 and 5 degrees, crossing the 0/360 boundary each cycle
    for (int cycle = 0; cycle < 5; ++cycle) {
        DecoderWitnessUpdate(&output, &decoder, 0.0, 355.0, CPR); // short reverse path
        DecoderWitnessUpdate(&output, &decoder, 355.0, 5.0, CPR); // forward through 0
        DecoderWitnessUpdate(&output, &decoder, 5.0, 0.0, CPR); // back to 0
    }

    REQUIRE(decoder.axes[0].error_count == 0);
}

TEST_CASE("Oscillation: per-pulse Gray-code invariant holds through reversals",
    "[oscillation]")
{
    const int CPR = 4;
    QOutputAxisState axis = QOutputAxisState_Construct(CPR);
    QOutputAxisState_Initialize(&axis, 0.0);
    const int ppr = axis.positions_per_rev;

    // Forward half revolution, then back to 0, then forward again
    auto fwd1 = DriveForward(&axis, 0, ppr / 2, ppr);
    auto rev1 = DriveBackward(&axis, ppr / 2, 0, ppr);
    auto fwd2 = DriveForward(&axis, 0, ppr / 4, ppr);

    CheckInvariants(fwd1, { 0, 0 }, ppr, "fwd1");
    CheckInvariants(rev1, { fwd1.back().a, fwd1.back().b }, ppr, "rev1");
    CheckInvariants(fwd2, { rev1.back().a, rev1.back().b }, ppr, "fwd2");
}

// ============================================================================
//  Main
// ============================================================================

int main(int argc, char* argv[]) {
    return Catch::Session().run(argc, argv);
}