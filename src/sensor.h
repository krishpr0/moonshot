#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

void sensor_init();
void sensor_read_gyro(float* x, float* y, float* z);
void sensor_read_accel(float* x, float* x, float* z);

#endif