#include "MEKF.h"
#include <math.h>

using namespace Eigen;

/**
 * @brief Default constructor.
 * @details Initializes the nominal quaternion to identity and bias to zero.
 */
MEKF::MEKF() {
    resetState(false); // Default to identity (upright)
    
    // Default Process Noise (Gyro noise and bias drift)
    // 0.001 is much more stable for flight than Identity[1.0]
    _proc = Matrix<float, 6, 6>::Identity() * 0.001f;
    
    // Default Error Covariance (Initial uncertainty)
    _cov = Matrix<float, 6, 6>::Identity() * 0.1f;
}

/**
 * @brief Resets the internal state to initial values.
 * @param physicalZAxisDown If true, initializes with a 180-degree flip around the X-axis.
 */
void MEKF::resetState(bool physicalZAxisDown) {
    if (physicalZAxisDown) {
        // Start with a 180 degree flip around X axis: q = [0, 1, 0, 0]
        _quat << 0.0f, 1.0f, 0.0f, 0.0f;
    } else {
        // Standard identity: q = [1, 0, 0, 0]
        _quat << 1.0f, 0.0f, 0.0f, 0.0f;
    }
    _bias.setZero();
}

/**
 * @brief Initializes the filter with process and initial error covariances.
 * @param Q_in  Process noise covariance (6x6).
 * @param P0_in Initial error covariance (6x6).
 */
void MEKF::init(const Matrix<float, 6, 6>& Q_in, const Matrix<float, 6, 6>& P0_in) {
    _proc = Q_in;
    _cov  = P0_in;
    resetState(false);
}

void MEKF::setProcessNoise(float q_diagonal) {
    _proc = Matrix<float, 6, 6>::Identity() * q_diagonal;
}

/**
 * @brief Predict Step: Propagates the quaternion and error covariance using gyroscope data.
 * @param gyro Gyroscope reading in rad/s [x, y, z].
 * @param dt Delta time in seconds.
 */
void MEKF::predict(const Matrix<float, 3, 1>& gyro, float dt) {
    // 1. Unbias the gyro reading using current bias estimate
    Matrix<float, 3, 1> omega = gyro - _bias;

    // 2. Propagate the nominal quaternion
    Matrix<float, 3, 1> delta_theta = omega * dt;
    float angle = delta_theta.norm();

    Matrix<float, 4, 1> dq;
    if (angle > 1e-5f) {
        float half_angle = angle * 0.5f;
        float s = sinf(half_angle) / angle;
        dq(0) = cosf(half_angle);
        dq(1) = delta_theta(0) * s;
        dq(2) = delta_theta(1) * s;
        dq(3) = delta_theta(2) * s;
    } else {
        dq(0) = 1.0f;
        dq(1) = delta_theta(0) * 0.5f;
        dq(2) = delta_theta(1) * 0.5f;
        dq(3) = delta_theta(2) * 0.5f;
    }

    _quat = quatMultiply(_quat, dq);
    _quat.normalize();

    // 3. Construct the State Transition Jacobian (F)
    Matrix<float, 6, 6> F = Matrix<float, 6, 6>::Identity();
    F.block<3, 3>(0, 0) -= skewSymmetric(omega) * dt; 
    F.block<3, 3>(0, 3)  = -Matrix<float, 3, 3>::Identity() * dt; 

    // 4. Propagate Error Covariance: P = F * P * F^T + Q
    _cov = F * _cov * F.transpose() + _proc;
}

/**
 * @brief Update step: Corrects the state using a directional sensor (Accel or Mag).
 * @param measurement The raw sensor vector (unnormalized).
 * @param reference The reference vector in world frame (e.g., [0,0,1] for gravity).
 * @param R_meas Measurement noise covariance matrix for this sensor (3x3).
 */
void MEKF::updateMeasurement(const Matrix<float, 3, 1>& measurement,
                              const Matrix<float, 3, 1>& reference,
                              const Matrix<float, 3, 3>& R_meas) {

    if (measurement.squaredNorm() < 1e-6f) return;

    // 1. Calculate Expected Measurement in the Body Frame
    Matrix<float, 3, 3> C = quatToRotationMatrix(_quat); 
    Matrix<float, 3, 1> h = C.transpose() * reference.normalized();

    // 2. Construct Measurement Jacobian (H)
    Matrix<float, 3, 6> H = Matrix<float, 3, 6>::Zero();
    H.block<3, 3>(0, 0) = skewSymmetric(h);

    // 3. Kalman Gain Calculation
    Matrix<float, 3, 3> S = H * _cov * H.transpose() + R_meas;
    Matrix<float, 6, 3> K = _cov * H.transpose() * S.inverse();

    // 4. Calculate Innovation (Residual)
    Matrix<float, 3, 1> innovation = measurement.normalized() - h;

    // 5. Calculate Error State Correction
    Matrix<float, 6, 1> dx = K * innovation;
    Matrix<float, 3, 1> attitude_error = dx.segment<3>(0);
    Matrix<float, 3, 1> bias_error     = dx.segment<3>(3);

    // 6. *** MULTIPLICATIVE UPDATE ***
    Matrix<float, 4, 1> dq_update;
    dq_update(0) = 1.0f;
    dq_update(1) = attitude_error(0) * 0.5f;
    dq_update(2) = attitude_error(1) * 0.5f;
    dq_update(3) = attitude_error(2) * 0.5f;

    _quat = quatMultiply(_quat, dq_update);
    _quat.normalize();

    // Additive update for the gyro bias
    _bias += bias_error;

    // 7. Update Covariance (Joseph Form for numerical stability)
    Matrix<float, 6, 6> I6   = Matrix<float, 6, 6>::Identity();
    Matrix<float, 6, 6> I_KH = I6 - K * H;
    _cov = I_KH * _cov * I_KH.transpose() + K * R_meas * K.transpose();
}

/**
 * @brief Specialized update that ONLY affects Heading (Yaw), decoupled from Tilt.
 * @param mag_measure The raw magnetometer reading in body frame.
 * @param mag_ref     The magnetic reference vector in world frame.
 * @param r_yaw       Noise variance for the heading measurement.
 */
void MEKF::updateMagnetometerYaw(const Matrix<float, 3, 1>& mag_measure,
                               const Matrix<float, 3, 1>& mag_ref,
                               float r_yaw) {
    if (mag_measure.squaredNorm() < 1e-6f) return;

    // 1. Current Body-to-World Orientation
    Matrix<float, 3, 3> C = quatToRotationMatrix(_quat);

    // 2. Project Body-Mag into World Frame to find Measured Heading
    Matrix<float, 3, 1> m_world = C * mag_measure.normalized();
    float psi_meas = atan2f(m_world(1), m_world(0));
    float psi_ref  = atan2f(mag_ref(1), mag_ref(0));

    // 3. Innovation (Yaw Error)
    float innovation = psi_meas - psi_ref;
    while (innovation >  3.14159265f) innovation -= 6.28318531f; 
    while (innovation < -3.14159265f) innovation += 6.28318531f; 

    // 4. Construct Jacobian (H)
    // We observe the rotation error around the World-Z axis.
    // Mapping world-Z error into body error coordinates: h_body = C.row(2).
    Matrix<float, 1, 6> H = Matrix<float, 1, 6>::Zero();
    H(0, 0) = C(2, 0); 
    H(0, 1) = C(2, 1);
    H(0, 2) = C(2, 2);

    // 5. Kalman Gain Calculation (1D Update)
    float S = (H * _cov * H.transpose())(0,0) + r_yaw;
    Matrix<float, 6, 1> K = _cov * H.transpose() / S;

    // 6. Update State
    Matrix<float, 6, 1> dx = K * innovation;
    Matrix<float, 3, 1> attitude_error = dx.segment<3>(0);
    Matrix<float, 3, 1> bias_error     = dx.segment<3>(3);

    Matrix<float, 4, 1> dq_update;
    dq_update(0) = 1.0f;
    dq_update(1) = attitude_error(0) * 0.5f;
    dq_update(2) = attitude_error(1) * 0.5f;
    dq_update(3) = attitude_error(2) * 0.5f;

    _quat = quatMultiply(_quat, dq_update);
    _quat.normalize();
    _bias += bias_error;

    // 7. Update Covariance
    Matrix<float, 6, 6> I6 = Matrix<float, 6, 6>::Identity();
    _cov = (I6 - K * H) * _cov;
}
Matrix<float, 3, 3> MEKF::skewSymmetric(const Matrix<float, 3, 1>& v) const {
    Matrix<float, 3, 3> S;
    S <<   0.0f, -v(2),  v(1),
          v(2),   0.0f, -v(0),
         -v(1),   v(0),  0.0f;
    return S;
}

/**
 * @brief Hamilton product of two quaternions: q1 * q2.
 */
Matrix<float, 4, 1> MEKF::quatMultiply(const Matrix<float, 4, 1>& q1,
                                                const Matrix<float, 4, 1>& q2) const {
    Matrix<float, 4, 1> r;
    r(0) = q1(0)*q2(0) - q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3); // w
    r(1) = q1(0)*q2(1) + q1(1)*q2(0) + q1(2)*q2(3) - q1(3)*q2(2); // x
    r(2) = q1(0)*q2(2) - q1(1)*q2(3) + q1(2)*q2(0) + q1(3)*q2(1); // y
    r(3) = q1(0)*q2(3) + q1(1)*q2(2) - q1(2)*q2(1) + q1(3)*q2(0); // z
    return r;
}

/**
 * @brief Converts a quaternion to its equivalent rotation matrix R_world_from_body (body -> world).
 */
Matrix<float, 3, 3> MEKF::quatToRotationMatrix(const Matrix<float, 4, 1>& qv) const {
    float qw = qv(0), qx = qv(1), qy = qv(2), qz = qv(3);
    Matrix<float, 3, 3> R;
    R << 1.0f - 2.0f*(qy*qy + qz*qz),  2.0f*(qx*qy - qz*qw),        2.0f*(qx*qz + qy*qw),
         2.0f*(qx*qy + qz*qw),          1.0f - 2.0f*(qx*qx + qz*qz), 2.0f*(qy*qz - qx*qw),
         2.0f*(qx*qz - qy*qw),          2.0f*(qy*qz + qx*qw),        1.0f - 2.0f*(qx*qx + qy*qy);
    return R;
}
