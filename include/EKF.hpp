#pragma once
#include <Arduino.h>

struct EKFState {
    float roll = 0, pitch =0, yaw = 0;
    float alt = 0;
    float vx = 0, vy = 0;
    float lat = 0, lng = 0; 
};

class EKF {
    private:    EKFState state_;
    float P[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    float Q[4] = {0.01, 0.01, 0.01, 0.1};
    float R[3] = {0.1, 0.1, 1.0};

    public:

    void predict(float gx, float gy, float dt) {
        state_.roll += gx * dt;
        state_.pitch += gy * dt;
    }

    void updateIMU(float roll_acc, float pitch_acc, float alt_baro) {
        state_.roll = 0.98 * state_.roll + 0.02 * roll_acc;
        state_.pitch = 0.98 * state_.pitch+ 0.02 * pitch_acc;
        state_.alt = 0.9 * state_.alt + 0.1 * alt_baro;
    }

    void updateGPS(double lat, double lng, float alt) {
        state_.lat = lat;
        state_.lng = lng;
        state_.alt = 0.7 * state_.alt + 0.3 * alt;
    }

    EKFState get() const { return state_;}
} ;
