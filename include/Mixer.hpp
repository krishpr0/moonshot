#pragma once
struct MotorOutputs {
    float m1, m2, m3, m4;
};

class QuadMixer {
    public:
        static MotorOutputs mix(float throttle, float roll, float pitch, float yaw) {
            float t = throttle;
            MotorOutputs out;
            out.m1 = t + roll + pitch - yaw;
            out.m2 = t - roll + pitch + yaw;
            out.m3 = t + roll - pitch + yaw;
            out.m4 = t - roll - pitch - yaw;

            out.m1 = constrain(out.m1, 1000, 2000);
            out.m2 = constrain(out.m2, 1000, 2000);
            out.m3 = constrain(out.m3, 1000, 2000);
            out.m4 = constrain(out.m4, 1000, 2000);

            return out;
        }
}