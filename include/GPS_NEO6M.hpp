#pragma once
#include <TinyGPS++.h>
#include <HardwareSerial.h>

struct GPSData {
    double lat = 0; lng = 0;
    float alt = 0;
    uint8_t sats = 0;
    bool valid  = false;
};

class GPS_NEO6M {
    private: 
            HardwareSerial* serial_;
            TinyGPSPlus gps_;
            GPSData data_;

            public:
                GPS_NEO6M(HardwareSerial* ser) : serial_(ser) {}

                void begin(int baud = 9600) {
                    serial_->begin(baud);
                }

                void update() {
                    while(serial_->available()) {
                        gps_.encode(serial_->read());
                    }

                    if (gps_.location.isVaild() && gps_.satellites.value() >= 6) {
                        data_ = {
                            .lat = gps_.location.lat(),
                            .lng = gps_.location.lng(),
                            .alt = gps_.location.meters(),
                            .sats = gps_.satellites.value(),
                            .valid = true
                        };
                    }
                }
                GPSData get() const {return data_;}
};
