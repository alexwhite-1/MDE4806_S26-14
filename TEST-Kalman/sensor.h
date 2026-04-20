#ifndef SENSOR_H
#define SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

#endif
