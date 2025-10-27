#pragma once 
#include <Adafruit_BMP280.h>

class Baro_BMP280 {
        private: 
            Adafruit_BMP280 bmp_;
            float altitude_ = 0;
            float pressure_ = 0;
            float temp_ = 0;
            float ground_alt_ = 0;
            bool calibrated_ = false;

            public:
            bool begin() {
                return bmp_.begin(0x76) || bmp_.begin(0x77);
            }

            void update() {
                pressure_ = bmp_.readPressure() / 100.0F;
                temp_ = bmp_.readTemperature();
                if (calibrated_) {
                    altitude_= bmp_.readAltitude(1013.25) - ground_alt_;
                }
            }

            void calibrateGround() {
                ground_alt_ = bmp_.readAltitude(1013.25);
                calibrated_ = true;
            }

            float getAltitude() const { return altitude_; } 
            float getPressure() const { return pressure_; }
};