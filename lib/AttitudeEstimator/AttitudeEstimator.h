#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <Arduino.h>
#include "IMUSensor.h"
#include "MEKF.h"

/**
 * @enum AttitudeFilterSel
 * @brief Available filters for attitude estimation.
 */
enum class AttitudeFilterSel {
    NONE,     ///< Simple integration (High drift)
    MADGWICK, ///< optimized gradient descent filter
    MAHONY,   ///< Proportional-Integral feedback filter
    EKF,      ///!Not implemented yet: Extended Kalman Filter 
    MEKF,     ///< Multiplicative Extended Kalman Filter
    NAV_MEKF, ///< 15-State Navigation MEKF
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
        * @brief Gets the currently selected orientation filter.
        * @return The currently selected AttitudeFilterSel.
        */
        AttitudeFilterSel getCurrentFilter() const { return _filterSel; }

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

        void resetOrientation();
        void resetEstimatorState();

        /**
         * @brief Manually sets the internal quaternion.
         * @details Useful when an external filter (like NavMEKF) provides the orientation.
         */
        void setQuaternion(float w, float x, float y, float z);

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
        float getTilt() const;

        /**
        * @brief Computes Z-axis acceleration in the Earth frame, with gravity removed.
        * @param isZDown True if the sensor is physically mounted Z-Down (subtracts -1G gravity instead of +1G).
        * @return Vertical acceleration in m/s^2.
        */
        float getNetVerticalAcceleration() const;

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
        bool getUseMagnetometer() const { return _useMagnetometer; }
        void getMagneticReference(float &mx, float &my, float &mz) const { mx = _mag_ref_x; my = _mag_ref_y; mz = _mag_ref_z; }
        /**
        * @brief Returns the underlying IMU sensor logic
        */
        IMUSensor* getIMU() const { return _imu; }

        /**
         * @brief Updates the noise parameters for the MEKF.
         * @param q_proc   Process noise (gyro drift). Lower is smoother.
         * @param r_accel  Accelerometer measurement noise. Higher trusts gyro more.
         * @param r_mag    Magnetometer measurement noise. Higher ignores magnetic jitter.
         */
        void setMEKFTuning(float q_proc, float r_accel, float r_mag);

        /**
         * @brief Sets the magnetic reference vector directly.
         * @param mx, my, mz Components of the local magnetic field in the World Frame (will be normalized).
         */
        void setMagneticReference(float mx, float my, float mz);

        /**
         * @brief Sets the magnetic reference based on a location preset.
         * @param location Index from MagLocation namespace (SAO_PAULO, MUNICH, etc.).
         */
        void setMagneticLocation(uint8_t location);

    private:
        IMUSensor* _imu;
        bool _useMagnetometer = true;
        
        volatile float _q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Internal Quaternion [w, x, y, z]

        // Filter Settings
        AttitudeFilterSel _filterSel = AttitudeFilterSel::MADGWICK;
        float _deltaT = 0.0f;

        // MEKF Tuning
        float _mekf_q_proc = 0.001f;
        float _mekf_r_accel = 0.1f;
        float _mekf_r_mag = 5.0f;

        // Magnetic Reference (World Frame)
        float _mag_ref_x = 1.0f; 
        float _mag_ref_y = 0.0f;
        float _mag_ref_z = 0.0f;

        // Madgwick parameters
        float _beta = sqrt(3.0f / 4.0f) * (PI * (1.0f / 180.0f)); 
        float _zeta = sqrt(3.0f / 4.0f) * (PI * (1.0f / 180.0f));
        bool _useZeta = false;
        float _w_bx = 0.0f, _w_by = 0.0f, _w_bz = 0.0f;
        float _magWeight = 0.05f; // Initial weight for magnetometer authority 

        // Mahony parameters
        float _Kp = 30.0f;
        float _Ki = 0.0f;
        float _ix = 0.0f, _iy = 0.0f, _iz = 0.0f; // Integral feedback terms

        // Fundamental Physical Constants
        static constexpr float _G_GRAVITY = 9.80665f;

        // Internal Math Functions
        void updateMadgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel);
        void updateMahony(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel);
        void updateEKF(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel);
        void updateMEKF(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, bool ignoreAccel);
        void updateNone(float ax, float ay, float az, float gx, float gy, float gz);

        MEKF _mekf; ///< MEKF filter instance
        
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
