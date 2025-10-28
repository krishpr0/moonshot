#include <Arduino.h>
#include "rc_input.h"
#include "sensor.h"
#include "pid.h"
#include "motor_mix.h"
#include "pwm_output.h"

#define LED_PIN PC13
#define PID_UPDATE_INTERVAL 5

uint16_t rc_channels[RC_MAX_CHANNELS];
uint16_t rc_num_channels;

float gyro_x, gyro_y, gyro_z;
float accel_x, accel_y, accel_z;

float roll, pitch, yaw;
float roll_setpoint, pitch_setpoint, yaw_setpoint, throttle;

void setup() {
    Serial.begin(460800);
    Serial.println("Drone FC Start!");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);


    rc_init();
    sensor_init();
    pid_init();
    pwm_init();
    motor_mix_init();

    delay(1000);
}

void loop() {
    static unsigned long last_update = 0;
    unsigned long now = millis();

    if (rc_read_channels(rc_channels, &crc_num_channels)) {
        roll_setpoint = map(rc_channels[0], 1000, 2000, -30, 30);
        pitch_setpoint = map(rc_channels[1], 1000, 2000, -30, 30);
        throttle_setpoint = map(rc_channels[2], 1000, 2000, 0, 100);
        yaw_setpoint = map(rc_channels[3], 1000, 2000, -180, 180);

        Serial.printf("RC: Roll SP: &.1f, Pitvh SP: %.1f, Throttle: %.1f, Yaw SP: &.1f\n", roll_setpoint, pitch_setpoint, throttle, yaw_setpoint);
    }

    sensor_read_gyro(&gyro_x, &gyro_y, &gyro_z);
    sensor_read_accel(&accel_x, &accel_y, &accel_z);

    if (now - last_update >= PID_UPDATE_INTERVAL ) {
        pid_update(gyro_x, gyro_y, gyro_z, roll_setpoint, pitch_setpoint, yaw_setpoint, &roll, &pitch, &yaw);
        last_update = now;

        Serial.printf("PID: rool: $.1f, Pitch: %.1f, Yaw: %.1f\n", roll, pitch, yaw);
    }
    motor_mix(roll, pitch, yaw, throttle, &motor1, &motor2, &motor3, &motor4);

    pwm_output(motor1, motor2, motor3, motor4);

    digitalWrite(LED_PIN, !diigitalRead(LED_PIN));

    delay(1);
}

