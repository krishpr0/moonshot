#include "pid.h"

#define KP 1.0f
#define KI 0.0f
#define KD 0.0f
#define PID_DT 0.005f

 float roll_integral = 0, pitch_integral = 0, yaw_integral = 0;
 float roll_prev_err = 0, pitch_prev_err = 0, yaw_prev_err = 0;

 void pid_init() {

 }

 void pid_update(float gyro_x, float gyro_y, float gyro_z, float roll_sp, float pitch_sp, float yaw_sp, float* roll, float* pitch, float* yaw) {
    float roll_err = roll_sp - gyro_x;
    float pitch_err = pitch_sp - gyro_y;
    float yaw_err = yaw_sp - gyro_z;

    roll_integral += roll_err * PID_DT;
    pitch_integral += pitch_err * PID_DT;
    yaw_integral += yaw_err * PID_DT;

    float roll_d = (roll_err - roll_prev_err) / PID_DT;
    float pitch_d = (pitch_err - pitch_prev_err) / PID_DT;
    float yaw_d = (yaw_err - yaw_prev_err) / PID_DT;

    *roll = KP * roll_err + KI * roll_integral + KD * roll_d;
    *pitch = KP * pitch_err + KI * pitch_integral + KD * pitch_d;
    *yaw = KP * yaw_err + KI  * yaw_integral + KD * yaw_d;

    roll_prev_err = roll_err;
    pitch_prev_err = pitch_err;
    yaw_prev_err = yaw_err;
    
 }