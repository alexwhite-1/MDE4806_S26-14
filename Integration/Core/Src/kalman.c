#include "kalman.h"

#include <math.h>

#define ROLL 0
#define PITCH 1
#define X_AXIS 2
#define Y_AXIS 3
#define Z_AXIS 4

// Static functions
void FillIdentity(float* matrix, int size) {
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            matrix[i * size + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

//
void kalman_run( float dt, StateVector* state_vector, ErrorCovarianceMatrix* error_covariance_matrix, const GyroSample* gyro, const AccelSample* accel, const ProcessNoiseMatrix* process_noise_matrix, const MeasurementNoiseMatrix* measurement_noise_matrix) {
	// GYRO PORTION
	CorrectedGyro correct_gyro = ComputeCorrectedValues(state_vector, gyro);
	TrigCache trig = ComputeTrigValues(state_vector->vector[ROLL], state_vector->vector[PITCH]);

	PredictionMatrix prediction_matrix = PredictionMatrix_Construct(&correct_gyro, &trig, dt);

	ComputeErrorCovarianceMatrix(error_covariance_matrix, &prediction_matrix, process_noise_matrix);

	// ACC PORTION
	MeasuredVector measured_vector = MeasuredVector_Construct(accel);
	ResidualErrorVector residual_error_vector =  ResidualErrorVector_Construct(state_vector, &measured_vector);

	// KALMAN CALC
	KalmanGainMatrix kalman_gain_matrix = KalmanGainMatrix_Construct(error_covariance_matrix, measurement_noise_matrix);

	// UPDATE STATE AND ERROR MATRIX
	UpdateStateVector(state_vector, &kalman_gain_matrix, &residual_error_vector, &correct_gyro, &trig, dt);
	UpdateErrorCovarianceMatrix(error_covariance_matrix, &kalman_gain_matrix);

	// DOUBLE OUTPUT
}

//=== Constructors ===//

/*
StateVector StateVector_Construct(float roll, float pitch, float gx, float gy, float gz) {
	StateVector state;

	state.vector[ROLL] = roll;
	state.vector[PITCH] = pitch;
	state.vector[X_AXIS] = gx;
	state.vector[Y_AXIS] = gy;
	state.vector[Z_AXIS] = gz;

	return state;
}
*/

StateVector StateVector_Construct() {
	StateVector state;

	for (int i = 0; i < MATRIX_SIZE; i++) {
		state.vector[i] = 0;
	}

	return state;
}

ProcessNoiseMatrix ProcessNoiseMatrix_Contruct() {
	ProcessNoiseMatrix noise;
    FillIdentity(&noise.matrix[0][0], MATRIX_SIZE); // 5x5
	return noise;
}

ErrorCovarianceMatrix ErrorCovarianceMatrix_Construct() {
    ErrorCovarianceMatrix covariance;
    FillIdentity(&covariance.matrix[0][0], MATRIX_SIZE); // 5x5
    return covariance;
}

MeasurementNoiseMatrix MeasurementNoiseMatrix_Contruct() {
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

	predict.matrix[1][0] = -gyro->Gy*trig->sin_r - gyro->Gz*trig->cos_r;
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


//=== Helper Functions ===//

void ComputeErrorCovarianceMatrix(ErrorCovarianceMatrix* error, const PredictionMatrix* predict, const ProcessNoiseMatrix* noise) {
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

void UpdateStateVector(StateVector* state, const KalmanGainMatrix* kalman, const ResidualErrorVector* residualerror, const CorrectedGyro* gyro, const TrigCache* trig, const float dt) {
	StateVector state_l, state_d;
	for (int i = 2; i < MATRIX_SIZE; i++) {
		state_l.vector[i] = state->vector[i];
	}

	// Calculate new predicted roll & pitch
	state_l.vector[ROLL] = state->vector[ROLL] + (gyro->Gx + gyro->Gy*trig->sin_r*trig->tan_p + gyro->Gz*trig->cos_r*trig->tan_p)*dt;
	state_l.vector[PITCH] = state->vector[PITCH] + (gyro->Gy*trig->cos_r - gyro->Gz*trig->sin_r)*dt;

	// Calculate delta x = Ky
	for (int i = 0; i < MATRIX_SIZE; i++) {
		state_d.vector[i] = kalman->matrix[i][0]*residualerror->vector[0] + kalman->matrix[i][1]*residualerror->vector[1];
	}

	// Update State vector (x- + delta x)
	for (int i = 0; i < MATRIX_SIZE; i++) {
		state->vector[i] = state_l.vector[i] + state_d.vector[i];
	}
}

void UpdateErrorCovarianceMatrix(ErrorCovarianceMatrix* error, const KalmanGainMatrix* kalman) {
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

CorrectedGyro ComputeCorrectedValues(const StateVector* state, const GyroSample* gyro) {
	CorrectedGyro correctedgyro;

	correctedgyro.Gx = gyro->gx - state->vector[X_AXIS];
	correctedgyro.Gy = gyro->gy - state->vector[Y_AXIS];
	correctedgyro.Gz = gyro->gz - state->vector[Z_AXIS];

	return correctedgyro;
}

TrigCache ComputeTrigValues(const float roll, const float pitch) {
	TrigCache trig;

	trig.cos_r = cosf(roll);
	trig.sin_r = sinf(roll);
	trig.tan_p = tanf(pitch);
	trig.sec2_p = 1 + (trig.tan_p*trig.tan_p);

	return trig;
}

// PsudoLookupTableFunctions ~ Will implement later
float LookupSin(float) {
	return 1;
}

float LookupCos(float) {
	return 1;
}

float LookupTan(float) {
	return 1;
}

