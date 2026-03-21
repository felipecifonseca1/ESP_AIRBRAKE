#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <Arduino.h>
#include "IMUSensor.h"

/**
 * @enum AttitudeFilterSel
 * @brief Available filters for attitude estimation.
 */
enum class AttitudeFilterSel {
    NONE,     ///< Simple integration (High drift)
    MADGWICK, ///< optimized gradient descent filter
    MAHONY,   ///< Proportional-Integral feedback filter
    EKF,      ///< Extended Kalman Filter (Reserved for future)
};

/**
 * @class AttitudeEstimator
 * @brief Handles sensor fusion and orientation mathematics for any IMUSensor.
 * @details Extracts raw data from the provided IMUSensor, runs it through
 *          Madgwick or Mahony filters, and calculates Roll, Pitch, Yaw, Tilt, 
 *          and Net Vertical Acceleration.
 */
class AttitudeEstimator {
    public:
        /**
        * @brief Constructor for the AttitudeEstimator.
        * @param imu Pointer to the abstract IMUSensor implementation (e.g., MPU9250_HAL).
        */
        AttitudeEstimator(IMUSensor* imu);

        /**
        * @brief Updates the filter with the latest IMU data.
        * @param dt Delta time since the last update in seconds.
        * @param ignoreAccel If true, the accelerometer correction is skipped (Gyro-only integration).
        */
        void update(float dt, bool ignoreAccel = false);

        // --- Configuration Getters & Setters ---
        /**
        * @brief Select which orientation filter to use.
        * @param sel Enum value for NONE, MADGWICK, or MAHONY.
        */
        void selectFilter(AttitudeFilterSel sel);

        /**
        * @brief Enable or disable the zeta-based drift learning (Madgwick only).
        * @param enabled True to enable.
        */
        void setDriftLearning(bool enabled);

        /**
        * @brief Set the filter's divergence/convergence rate.
        * @param errorDegPerSec Estimated gyroscope drift in degrees per second.
        */
        void setFilterBeta(float errorDegPerSec);

        /**
        * @brief Resets the internal orientation quaternion.
        * @param physicalZAxisDown If true, starts flipped 180deg (Z pointing down).
        */
        void resetOrientation(bool physicalZAxisDown);

        // --- Orientation Getters ---
        float getRoll() const;  ///< Returns Roll in degrees [-180, 180]
        float getPitch() const; ///< Returns Pitch in degrees [-90, 90]
        float getYaw() const;   ///< Returns Yaw in degrees [-180, 180]
        
        // Quaternion Getters
        float getQuaternionW() const { return _q[0]; } ///< Returns W component of quaternion
        float getQuaternionX() const { return _q[1]; } ///< Returns X component of quaternion
        float getQuaternionY() const { return _q[2]; } ///< Returns Y component of quaternion
        float getQuaternionZ() const { return _q[3]; } ///< Returns Z component of quaternion

        // --- Advanced Math Getters ---
        /**
        * @brief Calculates the rocket's tilt relative to the vertical.
        * @param physicalZAxisDown True if the IMU's Z-axis points towards the ground when upright.
        * @return Tilt angle in degrees [0° to 180°].
        */
        float getTilt(bool physicalZAxisDown) const;

        /**
        * @brief Calculates the net vertical acceleration in the Earth frame.
        * @return Net vertical acceleration in m/s^2 (excluding gravity).
        */
        float getNetVerticalAcceleration() const;

        /**
        * @brief Returns the underlying IMU sensor logic
        */
        IMUSensor* getIMU() const { return _imu; }

    private:
        IMUSensor* _imu;

        // Quaternions: [qW, qX, qY, qZ]
        float _q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

        // Filter Settings
        AttitudeFilterSel _filterSel = AttitudeFilterSel::MADGWICK;
        float _deltaT = 0.0f;

        // Madgwick parameters
        float _beta = sqrt(3.0f / 4.0f) * (PI * (1.0f / 180.0f)); 
        float _zeta = sqrt(3.0f / 4.0f) * (PI * (1.0f / 180.0f));
        bool _useZeta = false;
        float _w_bx = 0.0f, _w_by = 0.0f, _w_bz = 0.0f;

        // Mahony parameters
        float _Kp = 30.0f;
        float _Ki = 0.0f;
        float _ix = 0.0f, _iy = 0.0f, _iz = 0.0f; // Integral feedback terms

        // Fundamental Physical Constants
        static constexpr float _G_GRAVITY = 9.80665f;

        // Internal Math Functions
        void updateMadgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
        void updateMahony(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
        void updateEKF(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
        void updateNone(float ax, float ay, float az, float gx, float gy, float gz);
        
        float computeRoll() const;
        float computePitch() const;
        float computeYaw() const;
};

#endif // ATTITUDE_ESTIMATOR_H
