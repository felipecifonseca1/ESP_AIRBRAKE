#ifndef MEKF_H
#define MEKF_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;

/**
 * @class MEKF
 * @brief 15-State Multiplicative Extended Kalman Filter
 * @details State Vector Ordering:
 * [0:3]   Position Error (dp)
 * [3:6]   Velocity Error (dv)
 * [6:9]   Attitude Error (dtheta)
 * [9:12]  Accelerometer Bias (dab)
 * [12:15] Gyroscope Bias (dwb)
 */
class MEKF {
public:
    MEKF();
    ~MEKF() = default;

    /**
     * @brief Initializes the filter matrices
     */
    void init(const Matrix<float, 3, 1>& initial_pos,
              const Matrix<float, 3, 1>& initial_vel,
              const Matrix<float, 4, 1>& initial_quat, 
              float estimate_covariance, 
              float gyro_cov, float gyro_bias_cov, 
              float accel_proc_cov, float accel_bias_cov);

    /**
     * @brief Kinematic Integration & Covariance Prediction
     * @param gyro_meas Gyroscope reading [x, y, z] in rad/s
     * @param acc_meas Accelerometer reading [x, y, z] in m/s^2
     * @param dt Delta time in seconds
     */
    void predict(const Matrix<float, 3, 1>& gyro_meas, 
                 const Matrix<float, 3, 1>& acc_meas, 
                 float dt);

    /**
     * @brief Barometer Update (1D Z-Position)
     * @param measuredAltitude Altitude in meters
     * @param R_baro Measurement noise variance for barometer
     */
    void updateBaro(float measuredAltitude, float R_baro);

    /**
     * @brief Magnetometer Update (3D Attitude Correction)
     * @param mag_meas Raw magnetometer reading [x, y, z]
     * @param mag_ref Earth's magnetic field reference vector (normalized)
     * @param R_mag Measurement noise covariance (3x3)
     */
    void updateMag(const Matrix<float, 3, 1>& mag_meas, 
                   const Matrix<float, 3, 1>& mag_ref, 
                   const Matrix<float, 3, 3>& R_mag);

    /**
     * @brief GPS Update (6D Position & Velocity)
     * @param pos_meas GPS Position [x, y, z] in meters
     * @param vel_meas GPS Velocity [x, y, z] in m/s
     * @param R_gps Measurement noise covariance (6x6)
     */
    void updateGPS(const Matrix<float, 3, 1>& pos_meas, 
                   const Matrix<float, 3, 1>& vel_meas, 
                   const Matrix<float, 6, 6>& R_gps);

    // --- Getters ---
    Matrix<float, 3, 1> getPosition() const { return _pos; }
    Matrix<float, 3, 1> getVelocity() const { return _vel; }
    Matrix<float, 4, 1> getQuaternion() const { return _quat; }
    Matrix<float, 3, 1> getAccelBias() const { return _accel_bias; }
    Matrix<float, 3, 1> getGyroBias() const { return _gyro_bias; }

private:
    // Nominal States
    Matrix<float, 3, 1> _pos;
    Matrix<float, 3, 1> _vel;
    Matrix<float, 4, 1> _quat; 
    Matrix<float, 3, 1> _accel_bias;
    Matrix<float, 3, 1> _gyro_bias;

    // Filter Matrices
    Matrix<float, 15, 15> _cov;
    Matrix<float, 15, 15> _G;

    // Process Noise Sub-Matrices
    Matrix<float, 3, 3> _gyro_cov_mat;
    Matrix<float, 3, 3> _gyro_bias_cov_mat;
    Matrix<float, 3, 3> _accel_cov_mat;
    Matrix<float, 3, 3> _accel_bias_cov_mat;

    static constexpr float _G_GRAVITY = 9.80665f;

    // Mathematical Helpers
    Matrix<float, 15, 15> process_covariance(float dt);
    void injectErrorState(const Matrix<float, 15, 1>& error_state);
    
    Matrix<float, 3, 3> skewSymmetric(const Matrix<float, 3, 1>& v) const;
    Matrix<float, 4, 1> quatMultiply(const Matrix<float, 4, 1>& q1, const Matrix<float, 4, 1>& q2) const;
    Matrix<float, 3, 3> quatToMatrix(const Matrix<float, 4, 1>& q) const;
    Matrix<float, 3, 1> rotateInverse(const Matrix<float, 4, 1>& q, const Matrix<float, 3, 1>& v) const;
};

#endif // MEKF_H
