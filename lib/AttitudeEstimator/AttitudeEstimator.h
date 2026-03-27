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
        void update(float dt, bool ignoreAccel = false, bool physicalZAxisDown = false);

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
        * @brief Resets the internal orientation quaternion and clears drift biases.
        * @param physicalZAxisDown If true, starts flipped 180deg (Z pointing down).
        */
        void resetOrientation(bool unused = false);

        /**
         * @brief Resets the estimator state, including drift biases and integral terms.
         */
        void resetEstimatorState();

        // --- Orientation Getters ---
        float getRoll() const;  ///< Returns Roll in degrees [-180, 180]
        float getPitch() const; ///< Returns Pitch in degrees [-90, 90]
        float getYaw() const;   ///< Returns Yaw in degrees [-180, 180]
        float getBeta() const { return _beta; } ///< Diagnostic tracer
        
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
        float getTilt(bool unused = false) const;

        /**
        * @brief Computes Z-axis acceleration in the Earth frame, with gravity removed.
        * @param isZDown True if the sensor is physically mounted Z-Down (subtracts -1G gravity instead of +1G).
        * @return Vertical acceleration in m/s^2.
        */
        float getNetVerticalAcceleration(bool isZDown = false) const;

        /**
         * @brief Returns the transformed Z-acceleration entering the filter (diagnostic).
         */
        float getTransformedAccX() const { return _transformedAccX; }
        float getTransformedAccY() const { return _transformedAccY; }
        float getTransformedAccZ() const { return _transformedAccZ; }
        
        float getTransformedGyroX() const { return _transformedGyroX; }
        float getTransformedGyroY() const { return _transformedGyroY; }
        float getTransformedGyroZ() const { return _transformedGyroZ; }

        /**
         * @brief Enable or disable magnetometer fusion.
         * @param use True to enable.
         */
        void setUseMagnetometer(bool use);

        /**
        * @brief Adjusts the authority of the magnetometer in the fusion filter.
        * @param weight [0.0 to 1.0] A lower value (e.g., 0.05) prevents tilt oscillation.
        */
        void setMagnetometerWeight(float weight);
        /**
        * @brief Returns the underlying IMU sensor logic
        */
        IMUSensor* getIMU() const { return _imu; }

    private:
        IMUSensor* _imu;
        bool _useMagnetometer = true;
        
        volatile float _q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Internal Quaternion [w, x, y, z]

        // Filter Settings
        AttitudeFilterSel _filterSel = AttitudeFilterSel::MADGWICK;
        float _deltaT = 0.0f;
        float _magWeight = 0.05f; // Initial weight for magnetometer authority 

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

        float _transformedAccX = 0.0f;
        float _transformedAccY = 0.0f;
        float _transformedAccZ = 0.0f;

        float _transformedGyroX = 0.0f;
        float _transformedGyroY = 0.0f;
        float _transformedGyroZ = 0.0f;
};

#endif // ATTITUDE_ESTIMATOR_H
