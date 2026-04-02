#define MATRIX_SIZE 5
#define VECTOR_SIZE 2

typedef struct {
	float vector[MATRIX_SIZE];
} StateVector; // X

typedef struct {
	float vector[VECTOR_SIZE];
} MeasuredVector; // Z

typedef struct {
	float vector[VECTOR_SIZE];
} ResidualErrorVector; // Y

typedef struct {
	float matrix[MATRIX_SIZE][MATRIX_SIZE];
} PredictionMatrix; // F

typedef struct {
	float matrix[MATRIX_SIZE][MATRIX_SIZE];
} ErrorCovarianceMatrix; // P

typedef struct {
	float matrix[MATRIX_SIZE][MATRIX_SIZE];
} ProcessNoiseMatrix; // Q

typedef struct {
	float matrix[VECTOR_SIZE][VECTOR_SIZE];
} MeasurementNoiseMatrix; // R

typedef struct {
	float matrix[MATRIX_SIZE][VECTOR_SIZE];
} KalmanGainMatrix; // K

typedef struct {
	float gx;
	float gy;
	float gz;
} GyroSample;

typedef struct {
	float ax;
	float ay;
	float az;
} AccelSample;

typedef struct {
	float Gx;
	float Gy;
	float Gz;
} CorrectedGyro;

typedef struct {
	float sin_r;
	float cos_r;
	float tan_p;
	float sec2_p;
} TrigCache;

void kalman_run(
		float dt,
		StateVector* state_vector,
		ErrorCovarianceMatrix* error_covariance_matrix,
		const GyroSample* gyro,
		const AccelSample* accel,
		const ProcessNoiseMatrix* process_noise_matrix,
		const MeasurementNoiseMatrix* measurement_noise_matrix
);

//=== Constructors ===//

//StateVector StateVector_Construct(float roll, float pitch, float gx, float gy, float gz);

StateVector StateVector_Construct();

ProcessNoiseMatrix ProcessNoiseMatrix_Contruct();

ErrorCovarianceMatrix ErrorCovarianceMatrix_Construct();

MeasurementNoiseMatrix MeasurementNoiseMatrix_Contruct();

MeasuredVector MeasuredVector_Construct(const AccelSample* accl);

ResidualErrorVector ResidualErrorVector_Construct(const StateVector* state, const MeasuredVector* measured);

PredictionMatrix PredictionMatrix_Construct(const CorrectedGyro* gyro, const TrigCache* trig, const float dt);

KalmanGainMatrix KalmanGainMatrix_Construct(const ErrorCovarianceMatrix* error, const MeasurementNoiseMatrix* measurednoise);

//void Kalman_Run(StateVector* state, MeasuredVector* measured, float dt);

//=== Helper Functions ===//

void ComputeErrorCovarianceMatrix(ErrorCovarianceMatrix* error, const PredictionMatrix* predict, const ProcessNoiseMatrix* noise);

void UpdateStateVector(StateVector* state, const KalmanGainMatrix* kalman, const ResidualErrorVector* residualerror, const CorrectedGyro* gyro, const TrigCache* trig, const float dt);

void UpdateErrorCovarianceMatrix(ErrorCovarianceMatrix* error, const KalmanGainMatrix* kalman);

CorrectedGyro ComputeCorrectedValues(const StateVector* state, const GyroSample* gyro);

TrigCache ComputeTrigValues(const float roll, const float pitch);


float LookupSin(float);

float LookupCos(float);

float LookupTan(float);

