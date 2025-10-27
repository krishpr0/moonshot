#pragma once

enum class FlightMode {
    DISARMED,
    STABLIZE,
    ALT_HOLD,
    POS_HOLD,
    RTL
};

struct HomePoint {
    double lat = 0, lng = 0;
    bool set = false;
};

class ControlModes {
    public:     
        static FlightMode mode;
        static HomePoint home;

        static void setHome(double lat, double lng) {
            home.lat = lat; home.lng = lng; home.set = true;
        }
};

FlightMode ControlModes::mode = FlightMode::DISARMED;
HomePoint ControlModes::home;