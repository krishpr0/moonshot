#include "sensor.h"
#include <Wire.h>

#define MPU6050_ADDRESS 0x68
#define PWR_MGMT_1_REGISTER 0x6B
#define GYRO_DATA_REGISTER 0x43
#define ACCEL_DATA_REGISTER 0x3B
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define GYRO_SCALE  131.0f
#define ACCEL_SCALE 16384.0f

void sensor_init() {
  Wire.begin();
  Wire.setClock(400000);
  delay(250);
  
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(PWR_MGMT_1_REGISTER);
  Wire.write(0x00);
  Wire.endTransmiission();
  
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(ACCEL_CONFIG);
  Write.write(0x00);
  Wire.endTransmission();
}

void sensor_read_gyro(float* x, float* y, float* z) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(GYRO_DATA_REGISTER);
    Wire.requestFrom(MPU6050_ADDRESS, 6);
    int16_t raw_x = Wire.read() << 8 | Wire.read();
    int16_t raw_y = Wire.read() << 8 | Wire.read();
    int16_t raw_z = Wire.read() << 8 | Wire.read();

    *x = raw_x / GYRO_SCALE;
    *y = raw_y / GYRO_SCALE;
    *z = raw_z / GYRO_SCALE;

}

void sensor_read_accel(float* x, float* y, float* z) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(ACCEL_DATA_REGISTER);
    Wire.endTransmission();

    Wire.requestFrom(MPU6050_ADDRESS, 6);
    int16_t raw_x = Wire.read() << 8 | Wire.read();
    int16_t raw_y = Wire.read() << 8 | Wire.read();
    int16_t raw_z = Wire.read() << 8 | Wire.read();
    
    *x = raw_x / ACCEL_SCALE;
    *y = raw_y / ACCEL_SCALE;
    *z = raw_z / ACCEL_SCALE;
}