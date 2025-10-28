#ifndef PID_H
#define PID_H

#include <Arduino.h>

void pid_update(float gyro_x, float gyro_y, float gyro_z, float roll_sp, float pitch_sp, float yaw_sp, float* roll, float* pitch, float* yaw);

#endif