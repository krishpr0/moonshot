#pragma once

class PID {
    private: 
        float kp_, ki_, kd_;
        float integral = 0, prev_error = 0;
        float out_min_ = -1000, out_max = 1000;

        public:
            PID(float kp = 0, float ki = 0, float kd = 0) : kp_(kp), ki_(ki), kd_(kd) {}

            float compute(float sp, float pv, float dt) {
                float error = sp - pv;
                integral_ += error * dt;
                integral_ = constrain(integral_, out_min_ / (ki_ + 1e-6), out_max_ / (ki_ + 1e-6));
                float derivative  = (error - prev_error_) / dt;
                float out = kp_ * error + ki_ * integral_ + kd_ * derivative;
                prev_error_ = error;
                return constrain(out, out_min_, out_max_);
            }

            void setGains(float kp, float ki, float kd) {
                kp_ = kp; ki_ = ki; kd_ = kd;
            }

            void reset() {integral_ = 0; prev_error_ = 0;}
        };
        
}