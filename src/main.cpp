//ESP32 Quadcopter Moonshot Main File
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "../lib/crsf/crsf.h"

 #define LED_PIN 2
 #define LED_TOGGLE_INTERVAL 1000

 #define GYRO_INTERVAL 50
 #define PWR_MGMT_1_REGISTER 0x6B
 #define GYRO_DATA_REGISTER 0x43

 int16_t Gyro_X, Gyro_Y, Gyro_Z;
 float roll_rate = 0.0f, pitch_rate = 0.0f, yaw_rate = 0.0f;

 #define ELRS_INTERVAL 50
 #define RXD2 4
 #define TXD2 2
#define SBUS_BUFFER_SIZE 25

uint8_t _rcs_buf[SBUS_BUFFER_SIZE] {};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_count = 0;

HardwareSerial Serial2(2);

#define MOTOR1_PIN 25 //REAR LEFT
#define MOTOR2_PIN 26 //FRONT RIGHT
#define MOTOR3_PIN 27 //REAR RIGHT
#define MOTOR4_PIN 14 //FRONT LEFT

#define MOTOR1_CHANNEL 0
#define MOTOR2_CHANNEL 1
#define MOTOR3_CHANNEL 2
#define MOTOR4_CHANNEL 3

#define PWM_FREQ 50
#define PWM_RES  16
#define PWM_MIN_DUTY 3277
#define PWM_MAX_DUTY 6553

#define KP_ROLL_PITCH 0.8f
#define KI_ROLL_PITCH 0.05f
#define KD_ROLL_PITCH 0.1f

float i_roll = 0.0f, i_pitch = 0.0f, i_yaw = 0.0f;
float prev_e_roll = 0.0f, prev_e_pitch = 0.0f, prev_e_yaw = 0.0f;
float roll_out = 0.0f, pitch_out = 0.0f, yaw_out = 0.0f;
float roll_cmd = 0.0f, pitch_cmd = 0.0f, yaw_cmd = 0.0f, throttle_cmd = 0.0f, switch_st= 0.0f;

#define MAX_ROLL_PITCH_DEG 30.0f
#define MAX_YAW_RATE_DEG_S 200.0f
#define DEG2RAD (M_PU / 180.0f)
#define CRSF_MIN 172
#define CRSF_MAX 1811

bool armed = false;

void ledToggleLoop();
void gyroLoop();
void elrLoop();
void pidUpdate(float dt);
void motorMixing();
void setPWMPos(uint8_t channel, float percent);
void pwmSetup();

float map_ch(uint16_t val, float out_min, float out_max);

//Setup????????/??//////


void setup() {
    
    Serial.begin(460800);
    Serial.println("===Setup Starting!===");
     
    pinMode(LED_PIN, OUTPUT);

    //Gyro

    Wire.begin();
    Wire.setClock(400000);
    delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(PWR_MGMT_1_REGISTER);
    Wire.write(0x00);
    Wire.endTransmission();;


        //ELRS
    Serial2.begin(420000, SERIAL_8N1, RXD2, TXD2);

//PWM
    pwmSetup();

    Serial.println("===Setup Complete!===");        
}


//Main Loop//////
void loop() {
     unsigned long now = millis();
     static unsigned long last_pid = 0;
     float dt = (now - last_pid) / 1000.0f;
     last_pid = now;

     gyroLoop();
     elrsLoop();
     ledToggleLoop();
     pidUpdate(dt);
        motorMixing();

}

//Gyro Loop//////
long gyroNext = 0;
void gyroLoop() {
    if (millis() > gyroNext) {
        Wire.beginTransmission(0x68);
        Wire.write(GYRO_DATA_REGISTER);
        Wire.endTransmission();
        Wire.requestFrom(0x68, 6);

        Gyro_X = Wire.read() << 8 | Wire.read();
        Gyro_Y = Wire.read() << 8 | Wire.read();
        Gyro_Z = Wire.read() << 8 | Wire.read();

        //converting to rad/s 
        roll_rate = (Gyro_X / 32768.0f * 250.0f) * DEG2RAD;
        pitch_rate = (Gyro_Y / 32768.0f * 250.0f) * DEG2RAD;
        yaw_rate = (Gyro_Z / 32768.0f * 250.0f) * DEG2RAD;

        Serial.printf("GYRO - X: %d (%.2f rad/s), Y: %d (%.2f rad/s), Z: %d (%.2f rad/s)\n", 
                      Gyro_X, roll_rate, Gyro_Y, pitch_rate, Gyro_Z, yaw_rate);
                      gyroNext = millis() + GYRO_INTERVAL;
        }
    }

    //LED Toggle Loop//////
    long ledNext = 0;
    void ledToggleLoop() {
        if (millis() > ledNext) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            ledNext = millis() + LED_TOGGLE_INTERVAL;
        }
    }



    //ELRS Loop//////

    long elrsNext = 0;
    void elrsLoop() {
        if (millis() > elrsNext) { 
                if (Serial2.available()) {
                    size_t numBytes = Serial2.readBytes(_rcs_buf, SBUS_BUFFER_SIZE);
                    if (numBytes > 0) {
                        crsf_parse(&_rcs_buf[0], SBUS_BUFFER_SIZE, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS);

                        if (_raw_rc_count >= 5) {
                            roll_cmd = map_ch(_raw_rc_values[0], -MAX_ROLL_PITCH_DEG * DEG2RAD, MAX_ROLL_PITCH_DEG * DEG2RAD);
                            pitch_cmd = map_ch(_raw_rc_values[1], -MAX_ROLL_PITCH_DEG * DEG2RAD, MAX_ROLL_PITCH_DEG * DEG2RAD);
                            throttle_cmd = map_ch(_raw_rc_values[2], 0.0f, 1.0f);
                            yaw_cmd = map_ch(_raw_rc_values[3], -MAX_YAW_RATE_DEG_S * DEG2RAD, MAX_YAW_RATE_DEG_S * DEG2RAD);
                            switch_st = map_ch(_raw_rc_values[4], 0.0f, 1.0f);

                            armed = (switch_st > 0.5f);

                            Serial.printf("CH1: %d (R: %.2f); CH2: %d (P: %.2f); CH3: %d (T: %.2f); CH4: %d (Y: %.2f); CH5: %d (S: %.2f)\n",
                                          _raw_rc_values[0], roll_cmd,
                                          _raw_rc_values[1], pitch_cmd,
                                          _raw_rc_values[2], throttle_cmd,
                                          _raw_rc_values[3], yaw_cmd,
                                          _raw_rc_values[4], switch_st);
                                    }
                                }
                            } 
                             elrsNext = millis() + ELRS_INTERVAL;
                        }
                    }



    //PID Update//////

                    void pidUpdate(float dt) {
                        if (!armed) {
                        i_roll = i_pitch = i_yaw = 0.0f;
                        prev_p_roll = prev_e_pitch = prev_e_yaw = 0.0f;
                        roll_out = pitch_out = yaw_out = 0.0f;
                        return;
                    }

                    // For Roll    

                    float p_roll = roll_cmd - roll_rate;
                    i_roll += p_roll * dt;
                    i_roll = constrain(i_roll, -1.0f, 1.0f);

                    float d_roll = (p_roll - prev_p_roll) / dt;
                    roll_out = KP_ROLL_PITCH * p_roll + KI_ROLL_PITCH * i_roll + KD_ROLL_PITCH * d_roll;
                    roll_out = constrain(roll_out, -1.0f, 1.0f);
                    prev_p_roll = p_roll;


                    // For Pitch
                    float e_pitch = pitch_cmd - pitch_rate;
                    i_pitch += e_pitch * dt;
                    i_pitch = constrain(i_pitch, -1.0f, 1.0f);

                    float d_pitch = (e_pitch - prev_e_pitch) / dt;
                    pitch_out = KP_ROLL_PITCH * e_pitch + KI_ROLL_PITCH *i_pitch + KD_ROLL_PITCH * d_pitch;
                    pitch_out = constrain(pitch_out, -1.0f, 1.0f);
                    prev_e_pitch = e_pitch;


                    // For Yaw
                    float e_yaw = yaw_cmd - yaw_rate;
                    i_yaw += e_yaw * dt;
                    i_yaw = constrain(i_yaw, -1.0f, 1.0f);

                    float d_yaw = (e_yaw - prev_e_yaw) / dt;
                    yaw_out = KP_ROLL_PITCH * e_yaw + KI_ROLL_PITCH * i_yaw + KD_ROLL_PITCH * d_yaw;
                    yaw_out = constrain(yaw_out, -1.0f, 1.0f);
                    prev_e_yaw = e_yaw;
                }


    //Motor Mixing////// ( ITs for Quad X config)

    void motorMixing() {
        if (!armed) {
            setPWMPos(MOTOR1_CHANNEL, 0);
            setPWMPos(MOTOR2_CHANNEL, 0);
            setPWMPos(MOTOR3_CHANNEL, 0);
            setPWMPos(MOTOR4_CHANNEL, 0);
            return;
        }

        float thr = throttle_cmd;
        float roll = roll_out;
        float pitch = pitch_out;
        float yaw = yaw_out;


        float m1 = thr - roll - pitch - yaw; //REAR LEFT
        float m2 = thr + roll + pitch + yaw; //FRONT RIGHT
        float m3 = thr + roll - pitch - yaw; //REAR RIGHT
        float m4 = thr - roll + pitch + yaw; //FRONT LEFT

        float max_m = max({m1, m2, m3, m4});
        if (max_m > 1.0f) {
            m1 /= max_m;
            m2 /= max_m;
            m3 /= max_m;
            m4 /= max_m;
    }

     m1 = constrain(m1, 0.0f, 1.0f);
     m2 = constrain(m2, 0.0f, 1.0f);
     m3 = constrain(m3, 0.0f, 1.0f);
     m4 = constrain(m4, 0.0f, 1.0f);    


     setPWMPos(MOTOR1_CHANNEL, m1 * 100.0f);
     setPWMPos(MOTOR2_CHANNEL, m2 * 100.0f);
        setPWMPos(MOTOR3_CHANNEL, m3 * 100.0f);
        setPWMPos(MOTOR4_CHANNEL, m4 * 100.0f);
    }

    //Set PWM set//////

    void setPWMPos(uint8_t channel, float percent) {
        uint32_t duty = map(percent, 0, 100, PWM_MIN_DUTY, PWM_MAX_DUTY);
        ledcWrite(channel, duty);
    }


    //Channel Mapping//////

    float map_ch(uint16_t val, float out_min, float out_max) {
        val = constrain(val, CRSF_MIN, CRSF_MAX);
        return (val - CRSF_MIN) * (out_max - out_min) / (CRSF_MAX - CRSF_MIN) + out_min;
    }



