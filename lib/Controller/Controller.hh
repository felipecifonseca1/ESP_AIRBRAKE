#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cmath>


class Controller{
    public:
        Controller(float Kp, float Ki, float Kd, float mass_kg, float area_m2, float dt);
        ~Controller();

        float computePID(float setPoint, float input);
        void setLimits(float lowerLimit, float upperLimit);
        float clamp(float value);
        float antiWindUp(float integrative);
        float computeCd(float desiredApogee, float currentAltitude, float currentVelocity, float gravity, float rho);

    private:
    
        float _mass_kg;
        float _area_m2;

        // Ganhos e estado do PID
        float _Kp, _Ki, _Kd;
        float _dt;
        float _min_output, _max_output;
        float _windup_limit;
        float _integral;
        float _previous_error;
};

#endif