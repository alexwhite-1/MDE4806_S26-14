#ifndef SENSOR_H
#define SENSOR_H

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

#endif
