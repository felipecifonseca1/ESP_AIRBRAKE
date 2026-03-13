#ifndef BAROMETRIC_SENSOR_H
#define BAROMETRIC_SENSOR_H

#include <Arduino.h>

class BarometricSensor {
    protected:
        float groundPressureP0_Pa = 101325.0f; 
        float groundTemperatureT0_K = 298.15f;
        
        // Constants for altitude calculation
        static constexpr float L_ISA = 0.0065f;
        static constexpr float G_ACCEL = 9.80665f;
        static constexpr float M_AIR = 0.0289644f;
        static constexpr float R_GAS = 8.31447f;
        static constexpr float ISA_EXPONENT = (R_GAS * L_ISA) / (G_ACCEL * M_AIR);
        static constexpr float ALPHA_MOVING_AVERAGE = 0.01f;

    public:
        virtual ~BarometricSensor() = default;
        
        virtual bool init() = 0;
        virtual float getPressurePa() = 0;
        virtual float getTemperatureC() = 0;

        // Mathematical methods available to all barometers
        void calibrateGroundReference(int numReadings = 100);
        float altitudeFromPressure(float pressure_pa);
        float readAltitude();
        void recalibrateGroundPressure(float currentPressure_pa);
        
        // Setters and Getters
        void setGroundPressureP0(float p0);
        void setGroundTemperatureT0(float t0);
        float getGroundPressureP0() const;
        float getGroundTemperatureT0() const;
};

#endif // BAROMETRIC_SENSOR_H
