#ifndef BMP280_HAL_H
#define BMP280_HAL_H

#include "BarometricSensor.h"
#include <Adafruit_BMP280.h>

class BMP280_HAL : public BarometricSensor {
private:
    Adafruit_BMP280 bmp;
    uint8_t i2c_address;

public:
    BMP280_HAL(uint8_t address = 0x76) : i2c_address(address) {}

    bool init() override {
        if (!bmp.begin(i2c_address)) {
            return false;
        }

        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,  
                        Adafruit_BMP280::SAMPLING_X1,  
                        Adafruit_BMP280::SAMPLING_X4,  
                        Adafruit_BMP280::FILTER_OFF,   
                        Adafruit_BMP280::STANDBY_MS_1);
        return true;
    }

    float getPressurePa() override {
        return bmp.readPressure();
    }

    float getTemperatureC() override {
        return bmp.readTemperature();
    }
};

#endif // BMP280_HAL_H
