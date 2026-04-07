#ifndef MEKF_H
#define MEKF_H

#include <Arduino.h>
#include <ArduinoEigen.h>

using namespace Eigen;

/**
 * @class MEKF
 * @brief Multiplicative Extended Kalman Filter for attitude estimation.
 *
 * @details Estimates rocket orientation as a unit quaternion using gyroscope
 *          measurements for prediction and accelerometer/magnetometer measurements
 *          for correction.
 *
 *          Measurement updates use a sequential (one sensor at a time) strategy,
 *          which limits matrix inversion to 3x3, making this highly efficient on the ESP32.
 */
class MEKF {
public:
    /**
     * @brief Constructor for the MEKF.
     * @details Initializes the identity quaternion and zero bias.
     */
    MEKF();

    /**
     * @brief Virtual destructor for memory safety.
     */
    ~MEKF() = default;

    /**
     * @brief Initializes the MEKF with noise covariance matrices and resets state.
     * @param Q_in  Process noise covariance (6x6). Diagonal values tune gyro noise and bias drift.
     * @param P0_in Initial error covariance (6x6).
     */
    void init(const Matrix<float, 6, 6>& Q_in, const Matrix<float, 6, 6>& P0_in);

    /**
     * @brief Updates the process noise covariance matrix (Q).
     * @param q_diagonal Spectral density for body-fixed noise (diagonal scalar).
     */
    void setProcessNoise(float q_diagonal);

    /**
     * @brief Predict step: propagates the quaternion and error covariance using gyroscope data.
     * @param gyro Gyroscope reading in rad/s [gx, gy, gz].
     * @param dt   Delta time in seconds since the last predict call.
     */
    void predict(const Matrix<float, 3, 1>& gyro, float dt);

    /**
     * @brief Sequential measurement update step.
     * @details Call once for accelerometer, then once for magnetometer (if available).
     *          Each call inverts only a 3x3 matrix (S), keeping compute cost low.
     *
     * @param measurement The raw (unnormalized) sensor vector in the body frame (e.g., [ax, ay, az]).
     * @param reference   The reference vector in the Earth (world) frame.
     *                    E.g., [0, 0, 1] for gravity (Z-Up), [1, 0, 0] for magnetic North.
     * @param R_meas      Measurement noise covariance matrix for this sensor (3x3).
     */
    void updateMeasurement(const Matrix<float, 3, 1>& measurement,
                           const Matrix<float, 3, 1>& reference,
                           const Matrix<float, 3, 3>& R_meas);

    /**
     * @brief Specialized update that ONLY affects Heading (Yaw), decoupled from Tilt.
     * @param mag_measure The raw magnetometer reading in body frame.
     * @param mag_ref     The magnetic reference vector in world frame.
     * @param r_yaw       Noise variance for the heading measurement.
     */
    void updateMagnetometerYaw(const Eigen::Matrix<float, 3, 1>& mag_measure,
                               const Eigen::Matrix<float, 3, 1>& mag_ref,
                               float r_yaw);

    /** @brief Returns current orientation as a quaternion [w, x, y, z]. */
    Matrix<float, 4, 1> getQuaternion() const { return _quat; }

    /** @brief Returns current estimated gyroscope bias [x, y, z] in rad/s. */
    Matrix<float, 3, 1> getGyroBias()   const { return _bias; }

    /** 
     * @brief Resets state to identity.
     */
    void resetState();

private:
    // States
    Matrix<float, 4, 1> _quat;    // Nominal Quaternion [w, x, y, z]
    Matrix<float, 3, 1> _bias;    // Gyroscope Bias [x, y, z]

    // --- Covariance Matrices ---
    Matrix<float, 6, 6> _cov;  ///< Error state covariance (attitude error + bias error)
    Matrix<float, 6, 6> _proc; ///< Process noise covariance

    /**
     * @brief Constructs a 3x3 skew-symmetric (cross-product) matrix from a vector.
     * @param v Input 3D vector.
     * @return S such that S*u = v x u.
     */
    Matrix<float, 3, 3> skewSymmetric(const Matrix<float, 3, 1>& v) const;

    /**
     * @brief Hamilton product of two quaternions: q1 * q2.
     * @param q1 Left quaternion [w, x, y, z].
     * @param q2 Right quaternion [w, x, y, z].
     * @return Composed quaternion [w, x, y, z].
     */
    Matrix<float, 4, 1> quatMultiply(const Matrix<float, 4, 1>& q1, const Matrix<float, 4, 1>& q2) const;

    /**
     * @brief Converts a quaternion to its equivalent 3x3 Direction Cosine Matrix (body->world).
     * @param qv Quaternion [w, x, y, z].
     * @return Rotation matrix R such that v_world = R * v_body.
     */
    Matrix<float, 3, 3> quatToRotationMatrix(const Matrix<float, 4, 1>& qv) const;
};

#endif // MEKF_H
