#ifndef MOTOR_MIX_H
#define MOTOR_MIX_H

#include <Arduino.h>

void motor_mix_init();
void motor_mix(float roll, float pitch, float yaw, float throttle, float* m1, float* m2, float* m3, float* m4);

#endif