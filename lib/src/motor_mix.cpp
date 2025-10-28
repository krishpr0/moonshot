#include "motor_mix.h"

void motor_mix_init() {

}

void motor_mix(float roll, float pitch, float yaw,  float throttle, float* m1, float* m2, float* m3, float* m4) {
        *m1 = throttle + pitch + roll + yaw; //Front right
        *m2 = throttle + pitch - roll - yaw;
        *m3 = throttle - pitch + roll - yaw;
        *m4 = throttle - pitch - roll + yaw;

        *m1 = constrain(*m1, 0, 180);
        *m2 = constrain(*m2, 0, 180);
        *m3 = constrain(*m3, 0, 180);
        *m4 = constrain(*m4, 0, 180);
}