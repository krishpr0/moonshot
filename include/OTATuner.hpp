#pragma once 
#include "PID.hpp"
#include "MAVLinkHandler.hpp"

class OTATuner {
    private: 
        PID* roll_pid_;
        PID* pitch_pid_;
        PID* alt_pid_;
        MAVLinkHandler* mavlink_;

            public:
                OTATuner(PID* r, PID* p, PID* a, MAVLinkHandler* m)
                : roll_pid_(r), pitch_pid_(p), alt_pid_(a), mavlink_(m) {}

                void update() {
                    if (Serial.available()) {
                        String cmd = Serial.readStringUntil('\n');
                        if (cmd.startsWith("ROLL_P")) {
                            float p = cmd.substring(7).toFloat();
                            roll_pid_->setGains(p, roll_pid_->ki_, roll_pid_->kd_);
                            mavlink_->sendParam("ROLL_P", p);

                        }
                    }
                }
            };