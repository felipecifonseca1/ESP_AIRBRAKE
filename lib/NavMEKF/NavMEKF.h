#ifndef NAV_MEKF_H
#define NAV_MEKF_H

#include <Arduino.h>
#include <ArduinoEigen.h>
#include "Config_voo.h"


/**
 * @class NavMEKF
 * @brief 15-State Multiplicative Extended Kalman Filter
 * @details State Vector Ordering:
 * [0:3]   Position Error (dp)
 * [3:6]   Velocity Error (dv)
 * [6:9]   Attitude Error (dtheta)
 * [9:12]  Accelerometer Bias (dab)
 * [12:15] Gyroscope Bias (dwb)
 */
class NavMEKF {
public:
    NavMEKF();
    ~NavMEKF() = default;

    /**
     * @brief Initializes the filter matrices
     */
    void init(const Eigen::Matrix<float, 3, 1>& initial_pos,
              const Eigen::Matrix<float, 3, 1>& initial_vel,
              const Eigen::Matrix<float, 4, 1>& initial_quat, 
              float estimate_covariance, 
              float gyro_cov, float gyro_bias_cov, 
              float accel_proc_cov, float accel_bias_cov);

    /**
     * @brief Kinematic Integration & Covariance Prediction
     * @param gyro_meas Gyroscope reading [x, y, z] in rad/s
     * @param acc_meas Accelerometer reading [x, y, z] in m/s^2
     * @param dt Delta time in seconds
     */
    void predict(const Eigen::Matrix<float, 3, 1>& gyro_meas, 
                 const Eigen::Matrix<float, 3, 1>& acc_meas, 
                 float dt);

    /**
     * @brief Accelerometer Update
     * @param acc_meas Raw or aligned accelerometer reading [x, y, z] (G's)
     * @param R_acc Measurement noise covariance (3x3)
     */
    void updateAccel(const Eigen::Matrix<float, 3, 1>& acc_meas, 
                     const Eigen::Matrix<float, 3, 3>& R_acc);

    /**
     * @brief Barometer Update 
     * @param measuredAltitude Altitude in meters
     * @param R_baro Measurement noise variance for barometer
     */
    void updateBaro(float measuredAltitude, float R_baro);

    /**
     * @brief Magnetometer Update
     * @param mag_meas Raw magnetometer reading [x, y, z]
     * @param mag_ref Earth's magnetic field reference vector (normalized)
     * @param R_mag Measurement noise covariance (3x3)
     */
    void updateMag(const Eigen::Matrix<float, 3, 1>& mag_meas, 
                   const Eigen::Matrix<float, 3, 1>& mag_ref, 
                   float R_mag);

    /**
     * @brief Velocity Update
     * @param vel_meas Velocity vector [vx, vy, vz] (m/s)
     * @param R_vel Measurement noise covariance (3x3)
     */
    void updateVelocity(const Eigen::Matrix<float, 3, 1>& vel_meas, 
                        const Eigen::Matrix<float, 3, 3>& R_vel);

    /**
     * @brief GPS Update (6D Position & Velocity)
     * @param pos_meas GPS Position [x, y, z] in meters
     * @param vel_meas GPS Velocity [x, y, z] in m/s
     * @param R_gps Measurement noise covariance (6x6)
     */
    void updateGPS(const Eigen::Matrix<float, 3, 1>& pos_meas, 
                   const Eigen::Matrix<float, 3, 1>& vel_meas, 
                   const Eigen::Matrix<float, 6, 6>& R_gps);

    // --- Getters ---
    Eigen::Matrix<float, 3, 1> getPosition() const { return _pos; }
    Eigen::Matrix<float, 3, 1> getVelocity() const { return _vel; }
    Eigen::Matrix<float, 4, 1> getQuaternion() const { return _quat; }
    Eigen::Matrix<float, 3, 1> getAccelBias() const { return _accel_bias; }
    Eigen::Matrix<float, 3, 1> getGyroBias() const { return _gyro_bias; }

private:
    // Nominal States
    Eigen::Matrix<float, 3, 1> _pos;
    Eigen::Matrix<float, 3, 1> _vel;
    Eigen::Matrix<float, 4, 1> _quat; 
    Eigen::Matrix<float, 3, 1> _accel_bias;
    Eigen::Matrix<float, 3, 1> _gyro_bias;

    // Filter Matrices
    Eigen::Matrix<float, 15, 15> _cov;
    Eigen::Matrix<float, 15, 15> _G;

    // Process Noise Sub-Matrices
    Eigen::Matrix<float, 3, 3> _gyro_cov_mat;
    Eigen::Matrix<float, 3, 3> _gyro_bias_cov_mat;
    Eigen::Matrix<float, 3, 3> _accel_cov_mat;
    Eigen::Matrix<float, 3, 3> _accel_bias_cov_mat;

    static constexpr float _G_GRAVITY = G_GRAVITATIONAL_CONSTANT;

    // Mathematical Helpers
    void compute_process_covariance(float dt);
    void injectErrorState(const Eigen::Matrix<float, 15, 1>& error_state);
    
    Eigen::Matrix<float, 3, 3> skewSymmetric(const Eigen::Matrix<float, 3, 1>& v) const;
    Eigen::Matrix<float, 4, 1> quatMultiply(const Eigen::Matrix<float, 4, 1>& q1, const Eigen::Matrix<float, 4, 1>& q2) const;
    Eigen::Matrix<float, 3, 3> quatToMatrix(const Eigen::Matrix<float, 4, 1>& q) const;
    Eigen::Matrix<float, 3, 1> rotateInverse(const Eigen::Matrix<float, 4, 1>& q, const Eigen::Matrix<float, 3, 1>& v) const;

    // --- Scratch Matrices ---
    Eigen::Matrix<float, 15, 15> _I15;
    Eigen::Matrix<float, 15, 15> _F;
    Eigen::Matrix<float, 15, 15> _Q;
    Eigen::Matrix<float, 15, 15> _temp15x15;
    Eigen::Matrix<float, 15, 1>  _K15x1;
    Eigen::Matrix<float, 15, 3>  _K15x3;
    Eigen::Matrix<float, 15, 6>  _K15x6;
};

#endif // NAV_MEKF_H
