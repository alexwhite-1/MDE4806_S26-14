#include "kalman.h"

#include <math.h>
#ifdef DEBUG_KALMAN
	#include <stdio.h>
#endif // DEBUG_KALMAN

//=== Static Declarations ===//
static CorrectedGyro ComputeCorrectedValues(const StateVector* state, const GyroSample* gyro);
static TrigCache ComputeTrigValues(const float roll, const float pitch);
static void FillIdentity(float* matrix, int size);
static void ComputeErrorCovarianceMatrix(ErrorCovarianceMatrix* error, const PredictionMatrix* predict, const ProcessNoiseMatrix* noise);
static void UpdateStateVector(StateVector* state, const KalmanGainMatrix* kalman, const ResidualErrorVector* residualerror, const CorrectedGyro* gyro, const TrigCache* trig, const float dt);
static void UpdateErrorCovarianceMatrix(ErrorCovarianceMatrix* error, const KalmanGainMatrix* kalman);

//=== Kalman Filter ===//
void kalman_run(float dt, StateVector* state_vector, ErrorCovarianceMatrix* error_covariance_matrix, const GyroSample* gyro, const AccelSample* accel, const ProcessNoiseMatrix* process_noise_matrix, const MeasurementNoiseMatrix* measurement_noise_matrix) {
	// GYRO PORTION
	CorrectedGyro correct_gyro = ComputeCorrectedValues(state_vector, gyro);
	TrigCache trig = ComputeTrigValues(state_vector->vector[ROLL], state_vector->vector[PITCH]);
	#ifdef DEBUG_KALMAN
		printf("Trig: sin_r=%f, cos_r=%f, tan_p=%f, sec2_p=%f\n", trig.sin_r, trig.cos_r, trig.tan_p, trig.sec2_p);
	#endif // DEBUG_KALMAN

	PredictionMatrix prediction_matrix = PredictionMatrix_Construct(&correct_gyro, &trig, dt);
	#ifdef DEBUG_KALMAN
		printf("F matrix:\n");
		for (int i = 0; i < 2; i++) {
			printf("  [");
			for (int j = 0; j < 5; j++) {
				printf("%f, ", prediction_matrix.matrix[i][j]);
			}
			printf("]\n");
		}
	#endif // DEBUG_KALMAN

	ComputeErrorCovarianceMatrix(error_covariance_matrix, &prediction_matrix, process_noise_matrix);

	// ACC PORTION
	MeasuredVector measured_vector = MeasuredVector_Construct(accel);
	#ifdef DEBUG_KALMAN
		printf("Z = {%f, %f}\n", measured_vector.vector[0], measured_vector.vector[1]);
	#endif // DEBUG_KALMAN

	ResidualErrorVector residual_error_vector =  ResidualErrorVector_Construct(state_vector, &measured_vector);
	#ifdef DEBUG_KALMAN
		printf("Y = {%f, %f}\n", residual_error_vector.vector[0], residual_error_vector.vector[1]);
	#endif // DEBUG_KALMAN

	// KALMAN CALC
	KalmanGainMatrix kalman_gain_matrix = KalmanGainMatrix_Construct(error_covariance_matrix, measurement_noise_matrix);
	#ifdef DEBUG_KALMAN
		printf("K = \n");
		for (int i = 0; i < MATRIX_SIZE; i++) {
			printf("[%f, %f]\n", kalman_gain_matrix.matrix[i][0], kalman_gain_matrix.matrix[i][1]);
		}
	#endif // DEBUG_KALMAN

	// UPDATE STATE AND ERROR MATRIX
	UpdateStateVector(state_vector, &kalman_gain_matrix, &residual_error_vector, &correct_gyro, &trig, dt);
	UpdateErrorCovarianceMatrix(error_covariance_matrix, &kalman_gain_matrix);
}

//=== Constructors ===//
StateVector StateVector_Construct() {
	StateVector state;

	for (int i = 0; i < MATRIX_SIZE; i++) {
		state.vector[i] = 0;
	}

	return state;
}

ProcessNoiseMatrix ProcessNoiseMatrix_Construct() {
	ProcessNoiseMatrix noise;
    FillIdentity(&noise.matrix[0][0], MATRIX_SIZE); // 5x5
	return noise;
}

ErrorCovarianceMatrix ErrorCovarianceMatrix_Construct() {
    ErrorCovarianceMatrix covariance;
    FillIdentity(&covariance.matrix[0][0], MATRIX_SIZE); // 5x5
    return covariance;
}

MeasurementNoiseMatrix MeasurementNoiseMatrix_Construct() {
    MeasurementNoiseMatrix noise;
    FillIdentity(&noise.matrix[0][0], VECTOR_SIZE); // 2x2
    return noise;
}

MeasuredVector MeasuredVector_Construct(const AccelSample* accl) {
	MeasuredVector measured;

	measured.vector[ROLL] = atan2f(accl->ax, sqrtf(accl->ay*accl->ay + accl->az*accl->az));
	measured.vector[PITCH] = atan2f(-accl->ay, accl->az);

	return measured;
}

ResidualErrorVector ResidualErrorVector_Construct(const StateVector* state, const MeasuredVector* measured) {
	// NOTE: Function works when H = {{1,0,0,0,0},{0,1,0,0,0}}

	ResidualErrorVector residual;

	residual.vector[ROLL] = measured->vector[ROLL] - state->vector[ROLL];
	residual.vector[PITCH] = measured->vector[PITCH] - state->vector[PITCH];

	return residual;
}

PredictionMatrix PredictionMatrix_Construct(const CorrectedGyro* gyro, const TrigCache* trig, const float dt) {
	PredictionMatrix predict;

	// Create identity matrix
    FillIdentity(&predict.matrix[0][0], MATRIX_SIZE); // 5x5

	// Set prediction matrix values
	predict.matrix[0][0] += dt*trig->tan_p*(gyro->Gy*trig->cos_r - gyro->Gz*trig->sin_r);
	predict.matrix[0][1] = (gyro->Gy*trig->sin_r + gyro->Gz*trig->cos_r)*dt*trig->sec2_p;
	predict.matrix[0][2] = -dt;
	predict.matrix[0][3] = -trig->sin_r*trig->tan_p*dt;
	predict.matrix[0][4] = -trig->cos_r*trig->tan_p*dt;

	predict.matrix[1][0] = -(gyro->Gy*trig->sin_r + gyro->Gz*trig->cos_r)*dt;
	predict.matrix[1][3] = -trig->cos_r*dt;
	predict.matrix[1][4] = trig->sin_r*dt;

	return predict;
}

KalmanGainMatrix KalmanGainMatrix_Construct(const ErrorCovarianceMatrix* error, const MeasurementNoiseMatrix* measurednoise) {
	// NOTE: Function works when H = {{1,0,0,0,0},{0,1,0,0,0}}

	KalmanGainMatrix kalman;

	// Compute Innovation Covariance (S = HP(H^T)+R)
	float S[VECTOR_SIZE][VECTOR_SIZE];

	// Addition: O(4) Time, O(4) Space
	for (int i = 0; i < VECTOR_SIZE; i++) {
		for (int j = 0; j < VECTOR_SIZE; j++) {
			S[i][j] = error->matrix[i][j] + measurednoise->matrix[i][j];
		}
	}

	// Invert S
	const float eps = 1e-6f;
	float det = S[0][0]*S[1][1] - S[0][1]*S[1][0];

	if (fabsf(det) < eps) {
	    det = (det >= 0.f) ? eps : -eps;
	}

	float factor = 1.f / det;
	float temp = S[0][0];
	S[0][0] = S[1][1] * factor;
	S[1][1] = temp * factor;
	S[0][1] *= -1.f * factor;
	S[1][0] *= -1.f * factor;

	// Compute Kalman Gain (K=P(H^T)(S^-1))
	// : O(5) Time, O(1) Space
	for (int i = 0; i < MATRIX_SIZE; i++) {
		kalman.matrix[i][0] = error->matrix[i][0]*S[0][0] + error->matrix[i][1]*S[1][0];
		kalman.matrix[i][1] = error->matrix[i][0]*S[0][1] + error->matrix[i][1]*S[1][1];
	}

	return kalman;
}


//=== Static Helper Functions ===//

static CorrectedGyro ComputeCorrectedValues(const StateVector* state, const GyroSample* gyro) {
	CorrectedGyro correctedgyro;

	correctedgyro.Gx = gyro->gx - state->vector[X_AXIS];
	correctedgyro.Gy = gyro->gy - state->vector[Y_AXIS];
	correctedgyro.Gz = gyro->gz - state->vector[Z_AXIS];

	return correctedgyro;
}

static TrigCache ComputeTrigValues(const float roll, const float pitch) {
	TrigCache trig;

	const float eps = 1e-6f;
	const float safe_pitch = (fabsf(pitch) > (M_PI_2 - eps)) ? copysignf(M_PI_2 - eps, pitch) : pitch;

	trig.cos_r = cosf(roll);
	trig.sin_r = sinf(roll);
	trig.tan_p = tanf(safe_pitch);
	trig.sec2_p = 1 + (trig.tan_p*trig.tan_p);

	return trig;
}

static void FillIdentity(float* matrix, int size) {
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            matrix[i * size + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

static void ComputeErrorCovarianceMatrix(ErrorCovarianceMatrix* error, const PredictionMatrix* predict, const ProcessNoiseMatrix* noise) {
	float A[MATRIX_SIZE][MATRIX_SIZE] = {0};
	float B[MATRIX_SIZE][MATRIX_SIZE] = {0};

	// Multiplication: O(125) Time, O(25) Space
    for (int i = 0; i < MATRIX_SIZE; i++) { // Loop through each row of mat1
        for (int j = 0; j < MATRIX_SIZE; j++) { // Loop through each column of mat2
            for (int k = 0; k < MATRIX_SIZE; k++) { // Compute the dot product of row mat1[i] and column mat2[][j]
                A[i][j] += predict->matrix[i][k] * error->matrix[k][j];
            }
        }
    }

	// Multiplication: O(125) Time, O(25) Space
    for (int i = 0; i < MATRIX_SIZE; i++) { // Loop through each row of mat1
        for (int j = 0; j < MATRIX_SIZE; j++) { // Loop through each column of mat2
            for (int k = 0; k < MATRIX_SIZE; k++) { // Compute the dot product of row mat1[i] and column mat2[][j]
            	B[i][j] += A[i][k] * predict->matrix[j][k]; // Using Transpose of predict
            }
        }
    }

	// Addition: O(25) Time, O(1) Space
    for (int i = 0; i < MATRIX_SIZE; i++) {
    	for (int j = 0; j < MATRIX_SIZE; j++) {
    		error->matrix[i][j] = B[i][j] + noise->matrix[i][j];
    	}
    }
}

static void UpdateStateVector(StateVector* state, const KalmanGainMatrix* kalman, const ResidualErrorVector* residualerror, const CorrectedGyro* gyro, const TrigCache* trig, const float dt) {
	StateVector state_l, state_d;
	for (int i = 2; i < MATRIX_SIZE; i++) {
		state_l.vector[i] = state->vector[i];
	}

	// Calculate new predicted roll & pitch
	state_l.vector[ROLL] = state->vector[ROLL] + (gyro->Gx + gyro->Gy*trig->sin_r*trig->tan_p + gyro->Gz*trig->cos_r*trig->tan_p)*dt;
	state_l.vector[PITCH] = state->vector[PITCH] + (gyro->Gy*trig->cos_r - gyro->Gz*trig->sin_r)*dt;

	#ifdef DEBUG_KALMAN
		printf("state_l (predicted from motion) = {%f, %f, %f, %f, %f}\n",
			state_l.vector[0], state_l.vector[1], state_l.vector[2],
			state_l.vector[3], state_l.vector[4]);
	#endif // DEBUG_KALMAN

	// Calculate delta x = Ky
	for (int i = 0; i < MATRIX_SIZE; i++) {
		state_d.vector[i] = kalman->matrix[i][0]*residualerror->vector[0] + kalman->matrix[i][1]*residualerror->vector[1];
	}

	#ifdef DEBUG_KALMAN
		printf("state_d (K*Y correction) = {%f, %f, %f, %f, %f}\n",
			state_d.vector[0], state_d.vector[1], state_d.vector[2],
			state_d.vector[3], state_d.vector[4]);
	#endif // DEBUG_KALMAN

	// Update State vector (x- + delta x)
	for (int i = 0; i < MATRIX_SIZE; i++) {
		state->vector[i] = state_l.vector[i] + state_d.vector[i];
	}
}

static void UpdateErrorCovarianceMatrix(ErrorCovarianceMatrix* error, const KalmanGainMatrix* kalman) {
	// NOTE: Function works when H = {{1,0,0,0,0},{0,1,0,0,0}}

	float A[MATRIX_SIZE][MATRIX_SIZE] = {0};
	float B[MATRIX_SIZE][MATRIX_SIZE] = {0};

	// Compute (I - KH)
	for (int i = 0; i < MATRIX_SIZE; i++) {
		A[i][0] = -1.f * kalman->matrix[i][0];
		A[i][1] = -1.f * kalman->matrix[i][1];
		A[i][2] = 0;
		A[i][3] = 0;
		A[i][4] = 0;

		A[i][i] += 1.f;
	}

	// Multiplication: O(125) Time, O(25) Space
    for (int i = 0; i < MATRIX_SIZE; i++) { // Loop through each row of mat1
        for (int j = 0; j < MATRIX_SIZE; j++) { // Loop through each column of mat2
            for (int k = 0; k < MATRIX_SIZE; k++) { // Compute the dot product of row mat1[i] and column mat2[][j]
            	B[i][j] += A[i][k] * error->matrix[k][j];
            }
        }
    }

    // Copy: O(25) Time, O(1) Space
    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
			error->matrix[i][j] = B[i][j];
        }
    }
}
