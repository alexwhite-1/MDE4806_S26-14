#define CATCH_CONFIG_MAIN

#include <catch2/catch_all.hpp>
#include <cmath>

#include "kalman.h"

#define TOLERANCE 1e-5

bool StateEqual(const StateVector* s1, const float expected[MATRIX_SIZE], float tol = TOLERANCE) {
   for (int i = 0; i < MATRIX_SIZE; i++) {
      if (std::abs(s1->vector[i] - expected[i]) > tol) {
         return false;
      }
   }
   return true;
}

bool PDiagonalEqual(const ErrorCovarianceMatrix* P, const float expected[MATRIX_SIZE], float tol = TOLERANCE) {
   for (int i = 0; i < MATRIX_SIZE; i++) {
      if (std::abs(P->matrix[i][i] - expected[i]) > tol) {
         return false;
      }
   }
   return true;
}

// ============================================================================
// TEST 1: MOVING UPWARD (RAMP)
// ============================================================================
TEST_CASE("Test 1 - Moving Upward", "[test1]") {
   float dt = 0.01f;
   StateVector state = StateVector_Construct();
   ErrorCovarianceMatrix P = ErrorCovarianceMatrix_Construct();
   ProcessNoiseMatrix Q = ProcessNoiseMatrix_Construct();
   MeasurementNoiseMatrix R = MeasurementNoiseMatrix_Construct();

   GyroSample gyro = { 0.0f, 0.15f, 0.0f };
   AccelSample accel = { 0.0f, -0.2f, 0.98f };

   SECTION("Step 1") {
      kalman_run(dt, &state, &P, &gyro, &accel, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.f, 0.135714f, 0.f, -0.000671035f, 0.f };
      float expected_P_diag[MATRIX_SIZE] = { 0.666678f, 0.666678f, 1.99997f, 1.99997f, 2.f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }

   SECTION("Step 2") {
      kalman_run(dt, &state, &P, &gyro, &accel, &Q, &R);
      kalman_run(dt, &state, &P, &gyro, &accel, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.f, 0.178225f, 0.f, -0.00124499f, 0.f };
      float expected_P_diag[MATRIX_SIZE] = { 0.625078f, 0.625039f, 2.99976f, 2.99976f, 3.f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }

   SECTION("Step 3") {
      kalman_run(dt, &state, &P, &gyro, &accel, &Q, &R);
      kalman_run(dt, &state, &P, &gyro, &accel, &Q, &R);
      kalman_run(dt, &state, &P, &gyro, &accel, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.f, 0.194034f, 0.f, -0.00158578f, 0.f };
      float expected_P_diag[MATRIX_SIZE] = { 0.619179f, 0.619122f, 3.99919f, 3.99919f, 3.99998f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }
}

// ============================================================================
// TEST 2: MOVING DOWNWARD
// ============================================================================
TEST_CASE("Test 2 - Moving Downward", "[test2]") {
   float dt = 0.01f;
   StateVector state = StateVector_Construct();
   ErrorCovarianceMatrix P = ErrorCovarianceMatrix_Construct();
   ProcessNoiseMatrix Q = ProcessNoiseMatrix_Construct();
   MeasurementNoiseMatrix R = MeasurementNoiseMatrix_Construct();

   GyroSample gyro = { 0.0f, -0.15f, 0.0f };
   AccelSample accel = { 0.0f, 0.2f, 0.98f };

   SECTION("Step 1") {
      kalman_run(dt, &state, &P, &gyro, &accel, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.f, -0.135714f, 0.f, 0.000671035f, 0.f };
      float expected_P_diag[MATRIX_SIZE] = { 0.666678f, 0.666678f, 1.99997f, 1.99997f, 2.f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }

   SECTION("Step 2") {
      kalman_run(dt, &state, &P, &gyro, &accel, &Q, &R);
      kalman_run(dt, &state, &P, &gyro, &accel, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.f, -0.178225f, 0.f, 0.00124499f, 0.f };
      float expected_P_diag[MATRIX_SIZE] = { 0.625078f, 0.625039f, 2.99976f, 2.99976f, 3.f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }

   SECTION("Step 3") {
      kalman_run(dt, &state, &P, &gyro, &accel, &Q, &R);
      kalman_run(dt, &state, &P, &gyro, &accel, &Q, &R);
      kalman_run(dt, &state, &P, &gyro, &accel, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.f, -0.194034f, 0.f, 0.00158578f, 0.f };
      float expected_P_diag[MATRIX_SIZE] = { 0.619179f, 0.619122f, 3.99919f, 3.99919f, 3.99998f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }
}

// ============================================================================
// TEST 3: UP 2 STEPS, THEN DOWN 1
// ============================================================================
TEST_CASE("Test 3 - Up Then Down", "[test3]") {
   float dt = 0.01f;
   StateVector state = StateVector_Construct();
   ErrorCovarianceMatrix P = ErrorCovarianceMatrix_Construct();
   ProcessNoiseMatrix Q = ProcessNoiseMatrix_Construct();
   MeasurementNoiseMatrix R = MeasurementNoiseMatrix_Construct();

   GyroSample gyroUp = { 0.0f, 0.15f, 0.0f };
   AccelSample accelUp = { 0.0f, -0.2f, 0.98f };
   GyroSample gyroDown = { 0.0f, -0.15f, 0.0f };
   AccelSample accelDown = { 0.0f, 0.2f, 0.98f };

   SECTION("Step 1 (UP)") {
      kalman_run(dt, &state, &P, &gyroUp, &accelUp, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.f, 0.135714f, 0.f, -0.000671035f, 0.f };
      float expected_P_diag[MATRIX_SIZE] = { 0.666678f, 0.666678f, 1.99997f, 1.99997f, 2.f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }

   SECTION("Step 2 (UP)") {
      kalman_run(dt, &state, &P, &gyroUp, &accelUp, &Q, &R);
      kalman_run(dt, &state, &P, &gyroUp, &accelUp, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.f, 0.178225f, 0.f, -0.00124499f, 0.f };
      float expected_P_diag[MATRIX_SIZE] = { 0.625078f, 0.625039f, 2.99976f, 2.99976f, 3.f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }

   SECTION("Step 3 (DOWN)") {
      kalman_run(dt, &state, &P, &gyroUp, &accelUp, &Q, &R);
      kalman_run(dt, &state, &P, &gyroUp, &accelUp, &Q, &R);
      kalman_run(dt, &state, &P, &gyroDown, &accelDown, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.f, -0.0582455f, 0.f, 0.00435618f, 0.f };
      float expected_P_diag[MATRIX_SIZE] = { 0.619081f, 0.619122f, 3.99919f, 3.99919f, 3.99998f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }
}

// ============================================================================
// TEST 4: GRADUAL TILT UP
// ============================================================================
TEST_CASE("Test 4 - Gradual Tilt Up", "[test4]") {
   float dt = 0.01f;
   StateVector state = StateVector_Construct();
   ErrorCovarianceMatrix P = ErrorCovarianceMatrix_Construct();
   ProcessNoiseMatrix Q = ProcessNoiseMatrix_Construct();
   MeasurementNoiseMatrix R = MeasurementNoiseMatrix_Construct();

   SECTION("Step 1 (small tilt)") {
      GyroSample gyro = { 0.0f, 0.05f, 0.0f };
      AccelSample accel = { 0.0f, -0.05f, 0.999f };
      kalman_run(dt, &state, &P, &gyro, &accel, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.f, 0.0338394f, 0.f, -0.000166689f, 0.f };
      float expected_P_diag[MATRIX_SIZE] = { 0.666678f, 0.666678f, 1.99997f, 1.99997f, 2.f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }

   SECTION("Step 2 (medium tilt)") {
      GyroSample gyro1 = { 0.0f, 0.05f, 0.0f };
      AccelSample accel1 = { 0.0f, -0.05f, 0.999f };
      kalman_run(dt, &state, &P, &gyro1, &accel1, &Q, &R);

      GyroSample gyro2 = { 0.0f, 0.10f, 0.0f };
      AccelSample accel2 = { 0.0f, -0.15f, 0.98f };
      kalman_run(dt, &state, &P, &gyro2, &accel2, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.f, 0.108623f, 0.f, -0.00119944f, 0.f };
      float expected_P_diag[MATRIX_SIZE] = { 0.625045f, 0.625039f, 2.99976f, 2.99976f, 3.f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }

   SECTION("Step 3 (large tilt)") {
      GyroSample gyro1 = { 0.0f, 0.05f, 0.0f };
      AccelSample accel1 = { 0.0f, -0.05f, 0.999f };
      kalman_run(dt, &state, &P, &gyro1, &accel1, &Q, &R);

      GyroSample gyro2 = { 0.0f, 0.10f, 0.0f };
      AccelSample accel2 = { 0.0f, -0.15f, 0.98f };
      kalman_run(dt, &state, &P, &gyro2, &accel2, &Q, &R);

      GyroSample gyro3 = { 0.0f, 0.20f, 0.0f };
      AccelSample accel3 = { 0.0f, -0.30f, 0.95f };
      kalman_run(dt, &state, &P, &gyro3, &accel3, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.f, 0.23276f, 0.f, -0.00411049f, 0.f };
      float expected_P_diag[MATRIX_SIZE] = { 0.619164f, 0.619122f, 3.99919f, 3.99919f, 4.f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }
}

// ============================================================================
// TEST 5: ROLL AND PITCH COMBINATION
// ============================================================================
TEST_CASE("Test 5 - Roll and Pitch Combination", "[test5]") {
   float dt = 0.01f;
   StateVector state = StateVector_Construct();
   ErrorCovarianceMatrix P = ErrorCovarianceMatrix_Construct();
   ProcessNoiseMatrix Q = ProcessNoiseMatrix_Construct();
   MeasurementNoiseMatrix R = MeasurementNoiseMatrix_Construct();

   SECTION("Step 1 (roll right, pitch up)") {
      GyroSample gyro = { 0.10f, 0.08f, 0.02f };
      AccelSample accel = { 0.15f, -0.10f, 0.98f };
      kalman_run(dt, &state, &P, &gyro, &accel, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.101742f, 0.0685937f, -0.000503682f, -0.000338952f, 0.f };
      float expected_P_diag[MATRIX_SIZE] = { 0.666678f, 0.666678f, 1.99997f, 1.99997f, 2.f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }

   SECTION("Step 2 (roll left, pitch down)") {
      GyroSample gyro1 = { 0.10f, 0.08f, 0.02f };
      AccelSample accel1 = { 0.15f, -0.10f, 0.98f };
      kalman_run(dt, &state, &P, &gyro1, &accel1, &Q, &R);

      GyroSample gyro2 = { -0.12f, -0.06f, 0.01f };
      AccelSample accel2 = { -0.10f, 0.08f, 0.99f };
      kalman_run(dt, &state, &P, &gyro2, &accel2, &Q, &R);

      float expected_state[MATRIX_SIZE] = { -0.0257604f, -0.0252824f, 0.00126436f, 0.000971418f, -0.000010813f };
      float expected_P_diag[MATRIX_SIZE] = { 0.625031f, 0.625039f, 2.99976f, 2.99976f, 3.f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }

   SECTION("Step 3 (leveling out)") {
      GyroSample gyro1 = { 0.10f, 0.08f, 0.02f };
      AccelSample accel1 = { 0.15f, -0.10f, 0.98f };
      kalman_run(dt, &state, &P, &gyro1, &accel1, &Q, &R);

      GyroSample gyro2 = { -0.12f, -0.06f, 0.01f };
      AccelSample accel2 = { -0.10f, 0.08f, 0.99f };
      kalman_run(dt, &state, &P, &gyro2, &accel2, &Q, &R);

      GyroSample gyro3 = { 0.02f, 0.01f, -0.01f };
      AccelSample accel3 = { 0.02f, 0.01f, 1.0f };
      kalman_run(dt, &state, &P, &gyro3, &accel3, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.00275847f, -0.0157328f, 0.00058909f, 0.000744925f, -5.86203e-06f };
      float expected_P_diag[MATRIX_SIZE] = { 0.619121f, 0.619122f, 3.99919f, 3.99919f, 4.f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }
}

// ============================================================================
// TEST 6: DETECTING GYRO BIAS
// ============================================================================
TEST_CASE("Test 6 - Detecting Gyro Bias", "[test6]") {
   float dt = 0.01f;
   StateVector state = StateVector_Construct();
   ErrorCovarianceMatrix P = ErrorCovarianceMatrix_Construct();
   ProcessNoiseMatrix Q = ProcessNoiseMatrix_Construct();
   MeasurementNoiseMatrix R = MeasurementNoiseMatrix_Construct();

   SECTION("Step 1 (gyro drift, level accel)") {
      GyroSample gyro = { 0.05f, 0.08f, 0.03f };
      AccelSample accel = { 0.0f, 0.0f, 1.0f };
      kalman_run(dt, &state, &P, &gyro, &accel, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.0005f, 0.0008f, 0.f, 0.f, 0.f };
      float expected_P_diag[MATRIX_SIZE] = { 0.666678f, 0.666678f, 1.99997f, 1.99997f, 2.f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }

   SECTION("Step 2 (gyro drift continues)") {
      GyroSample gyro1 = { 0.05f, 0.08f, 0.03f };
      AccelSample accel1 = { 0.0f, 0.0f, 1.0f };
      kalman_run(dt, &state, &P, &gyro1, &accel1, &Q, &R);

      GyroSample gyro2 = { 0.06f, 0.09f, 0.04f };
      AccelSample accel2 = { 0.01f, -0.01f, 1.0f };
      kalman_run(dt, &state, &P, &gyro2, &accel2, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.00703767f, 0.00744995f, -0.0000831028f, -0.0000804919f, -2.24939e-08f };
      float expected_P_diag[MATRIX_SIZE] = { 0.625039f, 0.625039f, 2.99976f, 2.99976f, 3.f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }

   SECTION("Step 3 (actual rotation with bias)") {
      GyroSample gyro1 = { 0.05f, 0.08f, 0.03f };
      AccelSample accel1 = { 0.0f, 0.0f, 1.0f };
      kalman_run(dt, &state, &P, &gyro1, &accel1, &Q, &R);

      GyroSample gyro2 = { 0.06f, 0.09f, 0.04f };
      AccelSample accel2 = { 0.01f, -0.01f, 1.0f };
      kalman_run(dt, &state, &P, &gyro2, &accel2, &Q, &R);

      GyroSample gyro3 = { 0.15f, 0.12f, 0.03f };
      AccelSample accel3 = { 0.10f, -0.08f, 0.98f };
      kalman_run(dt, &state, &P, &gyro3, &accel3, &Q, &R);

      float expected_state[MATRIX_SIZE] = { 0.0669343f, 0.054465f, -0.0014749f, -0.00117274f, -2.21085e-06f };
      float expected_P_diag[MATRIX_SIZE] = { 0.619124f, 0.619122f, 3.99919f, 3.99919f, 4.f };

      REQUIRE(StateEqual(&state, expected_state));
      REQUIRE(PDiagonalEqual(&P, expected_P_diag));
   }
}