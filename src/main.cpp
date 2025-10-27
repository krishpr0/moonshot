#include <Arduino.h>
#include "PID.hpp"
#include "Mixer.hpp"
#include "IMU_MPU6050.hpp"
#include "Baro_BMP280.hpp"
#include "GPS_NEO6M.hpp"
#include "RC_CRSF.hpp"
#include "EKF.hpp"
#include "MAVLinkHandler.hpp"
#include "OTATuner.hpp"
#include "ControlModes.hpp"

HardwareSerial rcSerial(2);
HardwareSerial gpsSerial(1);

RC_CRSF rc(&rcSerial);
IMU_MPU6050 imu;
Baro_BMP280 baro;
GPS_NEO6M gps(&gpsSerial);
EKF ekf;
MAVLinkHandler mavlink;

PID rollPID(4.0, 0.1, 0.05);
PID pitchPID(4.0, 0.1, 0.05);
PID yawPID(3.0, 0.05, 0.0);
PID altPID(2.0, 0.5, 0.1);

OTATuner tuner(&rollPID, &pitchPID, &altPID, &mavlink);

unsigned long lastLoop = 0;
const int LOOP_RATE = 1000;

void setup() {
    Serial.begin(115200);
    rcSerial.begin(420000,  SERIAL_8N1, 16, 17);
    gpsSerial.begin(9600, SERIAL_8N1, 17, 16);
    rc.begin();

    if (!imu.begin()) while(1);
    if (!baro.begin()) while(1);

    gps.begin();
    delay(1000);

    baro.calibrateGround();
    mavlink.sendHeartbeat();
}

void loop() {
    unsigned long now = micros();
    if (now - lastLoop < 1e6 / LOOP_RATE) return;
    float dt = (now - lastLoop) / 1e6;
    lastLoop = now;

    rc.update();
    imu.update();
    baro.update();
    gps.update();
    tuner.update();

    auto r = rc.get();
    auto i = imu.get();
    auto g = gps.get();

    if (!r.valid || !i.valid) return;

    if (r.armed && ControModes::mode == FlightMode::DISARMED) {
        ControlModes::mode = FlightMode::STABLIZE;

        if (g.vaild) ControlModes::setHome(g.lat, g.lng);
    } else if (!r.armed) {
        ControlModes::mode = FlightMode::DISARMED;
        rollPID.reset(); pitchPID(); yawPID.reset(); altPID.reset();
    }

    float acc_roll = atan2(i.ay, i.az) * RAD_TO_DEG;
    float acc_pitch = atan2(-i.ax, sqrt(i.ay*i.ay + i.az*i,az)) * RAD_TO_DEG;
    ekf.predict(i.gx, i.gy, dt);
    ekf.updateIMU(acc_roll, acc_pitch, baro.getAltitude());
    if (g.valid) ekf.updateGPS(g.lat, g.lng, g.alt);
    auto e = ekf.get();

    float throttle = r.channels[2] * 1800 + 1800;
    float roll_sp = (r.channels[0] - 0.5) * 400;
    float pitch_sp = (r.channels[1] - 0.5) * 400;
    float yaw_sp = (r.channls[3] - 0.5) * 400;

    if (ControlModes::mode == FlightMode::ALT_HOLD) {
        float alt_sp = 2.0;
        throttle = 1500 + altPID.compute(alt_sp, e.alt, dt);
    }

    float roll_out = rollPID.compute(roll_sp, i.gx, dt);
    float pitch_out = pitchPID.compute(pitch_sp, i.gy, dt);
    float yaw_out = yawPID.compute(yaw_sp, i.gz, dt);

    auto motors = QuadMixer::mix(throttle, roll_out, pitch_out, yaw_out);

    if (millis() % 100 == 0) {
        mavlink.sendHeartbeat();
        mavlink.sendAttitude(e.roll, e.pitch, e.yaw);
        mavlink.sendParam("ROLL_P", rollPID.kp_);
    }

    Serial.printf("Mode: %d | Alt: %.2f | M: %.0f %.0f %.0f %.0f\n", (int)ControlModes::modes, e.alt, motors.m1, motors.m2, motors.m3, motors.m4);
    

}