#include "Controller.hh"

/**
 * @brief Constructor for the Controller class.
 * @details Initializes PID gains, rocket parameters, and time step.
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 * @param mass Mass of the rocket (kg).
 * @param area Reference area of the control surface (m²).
 * @param dt Time step for the controller (s).
 * * @param print If true, prints the saved values to Serial Monitor for debugging.
 */
Controller::Controller(float Kp, float Ki, float Kd, float mass, float area, float dt){
    _mass = mass;
    _area = area;
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _dt = dt;
    _min_output = 0;
    _max_output = 1;
    _windup_limit = 1.0; // Valor padrão, pode ser ajustado
    _integral = 0.0;
    _previous_error = 0.0;
}

/**
 * @brief Destructor for the Controller class.
 */
Controller::~Controller(){
}

/**
 * @brief Computes the PID controller output.
 * @details Implements a standard PID control algorithm with anti-windup for the integral term
 * @param setpoint Desired target value.
 * @param current_value Current measured value.
 * @return float Control output after applying PID algorithm.
 **/
float Controller::computePID(float setpoint, float current_value) {
    float error = setpoint - current_value;

    // 2. Calcula o termo integral com anti-windup
    _integral += error * _dt;
    if (_integral > _windup_limit) {
        _integral = _windup_limit;
    } else if (_integral < -_windup_limit) {
        _integral = -_windup_limit;
    }

    // 3. Calcula o termo derivativo
    float derivative = (error - _previous_error) / _dt;

    // 4. Calcula a saída PID total
    float output = (_Kp * error) + (_Ki * _integral) + (_Kd * derivative);

    // 5. Salva o erro atual para a próxima iteração do derivativo
    _previous_error = error;

    // 6. Satura (clamp) a saída para os limites definidos
    if (output > _max_output) {
        output = _max_output;
    } else if (output < _min_output) {
        output = _min_output;
    }

    return output;
}

/**
 * @brief Set the output limits for the controller.
 * @param lowerLimit Desired lower limit.
 * @param upperLimit Desired upper limit.
 **/
void Controller::setLimits(float lowerLimit, float upperLimit) {
    _min_output = lowerLimit; 
    _max_output = upperLimit; 
}

/**
 * @brief Clamp a value within the set output limits.
 * @note Max and min limits should be set prior to using this function (setLimits()).
 * @param value Value to be clamped.
 */
float Controller::clamp(float value) {
    if (value > _max_output) {
        return _max_output; 
    }
    if (value < _min_output) {
        return _min_output; 
    }
    return value;
}

/**
 * @brief Clamp the integrative term to prevent windup.
 * @param integrative Current value of the integrative term.
 */
float Controller::antiWindUp(float integrative){
    if (integrative > _windup_limit)
        return _windup_limit;
    if (integrative < -_windup_limit)
        return -_windup_limit;
    return integrative;
}

/**
 * @brief Computes the desired drag coefficient (Cd) using inverse dynamics based on the current state.
 * @param desiredApogee Target apogee altitude (m).
 * @param currentAltitude Current altitude of the rocket (m).
 * @param currentVelocity Current vertical velocity of the rocket (m/s).
 * @param gravity Local gravitational acceleration (m/s²).
 * @param rho Air density at current altitude (kg/m³).
 * @param integrative Current value of the integrative term.
 */
float Controller::computeCd(float desiredApogee, float currentAltitude, float currentVelocity, float gravity, float rho){
    float divisor1 = 1;
    float divisor2 = 1;
    if (2*(desiredApogee - currentAltitude) < 1) {
        divisor1 = 1;
    }
    else {
        divisor1 = 2*(desiredApogee - currentAltitude);
    }
    float acc_desejado = - currentVelocity*currentVelocity / divisor1;
    if (_area*rho*(currentVelocity*currentVelocity) < 1) {
        divisor2 = 1;
    }
    else {
        divisor2 = _area*rho*(currentVelocity*currentVelocity);
    }
    float Cd_desired = - 2*_mass*(acc_desejado - gravity)/divisor2;
    if (Cd_desired < 0 || acc_desejado > 0) {
        return 0;
    }
    if (Cd_desired > 1.5){
        return 1.5;
    }
    if (currentVelocity < 230 and currentVelocity > 60) {
        float x = 1 + 2*(Cd_desired-1)/3 - 1*(Cd_desired-1)*(Cd_desired-1)/9 + 4*(Cd_desired-1)*(Cd_desired-1)*(Cd_desired-1)/81;
        if (x < Cd_desired) {
            return Cd_desired;
        }
        return x;
    }

    return Cd_desired;
}