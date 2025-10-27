#pragma once
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

struct IMUData {
    float roll, pitch, yaw;
    float gx, gy, gz;
    float ax, ay, az;
    float temp;
    bool valid = false;
    unsigned long ts = 0;
};

class IMU_MPU6050 {
    private:    
            Adafruit_MPU6050 mpu_;
            IMUData data_;
            float roll_ = 0, pitch_ = 0, yaw_ = 0;
            float bias_[3] = {0};
            float alpha_ = 0.98;

            void calibrate(int samples = 1000) {
                Serial.println("IMU calibrating... Keep flat!!!!!");
                float sum[3] = {0};
                for (int i = 0; i < samples; i+) {
                    sensors_event_t a, g, t; mpu_.getEvent(&a, &g, &t);
                    sum[0] += g.gyro.x; sum[1] += g.gyro.y; sum[2] += g.gyro.z;
                    delay(2);
                }
                    for (int i = 0; i < 3; i++) bias_[i] = sum[i] / samples;
                    Serial.println("IMU Calibrated");
            }   

            public: 
                bool begin() {
                    if (!mpu_.begin()) return  false;
                    mpu_.setAccelerometerRange(MPU6050_RANGE_8_G);
                    mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
                    mpu_.SetFilterBandwidth(MPU6050_BAND_21_HZ);
                    calibrate();
                    return true;
                } 

                void update() {
                    sensors_event_t a, g, t;
                    mpu_.getEvent(&a, &g, &t);

                    float gx = g.gyro.x - bias_[0];
                    float gy = g.gyro.y - bias_[1];
                    float gz = g.gyro.z - bias_[2];

                    float dt = 0.01f;

                    float acc_roll = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
                    float acc_pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;


                    roll_ = alpha_ * (roll_ + gx * dt) + (1 - alpha_) * acc_roll;
                    pitch_ = alpha_ * (pitch_ + gy * dt) + (1 - alpha_) * acc_pitch;
                    yaw_ += gz * dt;


                    data_ = {
                        .roll = roll_, .pitch = pitch_, .yaw = yaw_, .gx = gx, .gy = gy, .gz  = gz, .ax = a.acceleration.x, .ay = a.acceleration.y, .az = a.acceleration.z, .temp = t.temperature, .valid = true, .ts = millis()
                    };
                }

                IMUData get() const { return data_;}
}