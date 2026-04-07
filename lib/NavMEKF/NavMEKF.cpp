#include "NavMEKF.h"

using namespace Eigen;

NavMEKF::NavMEKF() {}

void NavMEKF::init(const Matrix<float, 3, 1>& initial_pos,
                const Matrix<float, 3, 1>& initial_vel,
                const Matrix<float, 4, 1>& initial_quat, 
                float estimate_covariance, 
                float gyro_cov, float gyro_bias_cov, 
                float accel_proc_cov, float accel_bias_cov) {
    
    _pos = initial_pos;
    _vel = initial_vel;
    _quat = initial_quat;
    
    _accel_bias.setZero();
    _gyro_bias.setZero();

    _cov = estimate_covariance * Matrix<float, 15, 15>::Identity();
    _G = Matrix<float, 15, 15>::Zero();
    
    // Constant Transition mappings (dp_dot/dv and dtheta_dot/dwb)
    _G.block<3, 3>(0, 3) = Matrix<float, 3, 3>::Identity(); 
    _G.block<3, 3>(6, 12) = -Matrix<float, 3, 3>::Identity(); 

    _I15 = Matrix<float, 15, 15>::Identity();
    _Q = Matrix<float, 15, 15>::Zero();
    _F = Matrix<float, 15, 15>::Identity();

    _gyro_cov_mat = gyro_cov * Matrix<float, 3, 3>::Identity();
    _gyro_bias_cov_mat = gyro_bias_cov * Matrix<float, 3, 3>::Identity();
    _accel_cov_mat = accel_proc_cov * Matrix<float, 3, 3>::Identity();
    _accel_bias_cov_mat = accel_bias_cov * Matrix<float, 3, 3>::Identity();
}

void NavMEKF::compute_process_covariance(float dt) {
    _Q.setZero();
    float dt2 = dt * dt; float dt3 = dt2 * dt; 
    float dt4 = dt3 * dt; float dt5 = dt4 * dt;

    // --- Accel & Accel Bias Block (Indices 0:6 and 9:12) ---
    _Q.block<3, 3>(3, 3) = _accel_cov_mat * dt + _accel_bias_cov_mat * (dt3 / 3.0f);
    _Q.block<3, 3>(3, 0) = _accel_cov_mat * (dt2 / 2.0f) + _accel_bias_cov_mat * (dt4 / 8.0f);
    _Q.block<3, 3>(0, 3) = _accel_cov_mat * (dt2 / 2.0f) + _accel_bias_cov_mat * (dt4 / 8.0f);
    _Q.block<3, 3>(3, 9) = -_accel_bias_cov_mat * (dt2 / 2.0f);
    _Q.block<3, 3>(9, 3) = -_accel_bias_cov_mat * (dt2 / 2.0f);
    _Q.block<3, 3>(0, 0) = _accel_cov_mat * (dt3 / 3.0f) + _accel_bias_cov_mat * (dt5 / 20.0f);
    _Q.block<3, 3>(0, 9) = -_accel_bias_cov_mat * (dt3 / 6.0f);
    _Q.block<3, 3>(9, 0) = -_accel_bias_cov_mat * (dt3 / 6.0f);
    _Q.block<3, 3>(9, 9) = _accel_bias_cov_mat * dt;

    // --- Gyro & Gyro Bias Block (Indices 6:9 and 12:15) ---
    _Q.block<3, 3>(6, 6)   = _gyro_cov_mat * dt + _gyro_bias_cov_mat * (dt3 / 3.0f);
    _Q.block<3, 3>(6, 12)  = -_gyro_bias_cov_mat * (dt2 / 2.0f);
    _Q.block<3, 3>(12, 6)  = -_gyro_bias_cov_mat * (dt2 / 2.0f);
    _Q.block<3, 3>(12, 12) = _gyro_bias_cov_mat * dt;
}

void NavMEKF::predict(const Matrix<float, 3, 1>& gyro_meas, const Matrix<float, 3, 1>& acc_meas, float dt) {
    Matrix<float, 3, 1> gyro_clean = gyro_meas - _gyro_bias;
    Matrix<float, 3, 1> acc_clean = acc_meas - _accel_bias;

    // 1. Integrate Quaternion (Using Exponential Map for stability at high spin)
    Matrix<float, 3, 1> delta_theta = gyro_clean * dt;
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

    // 2. Integrate Position and Velocity (State Propagation)
    Matrix<float, 3, 3> R = quatToMatrix(_quat);
    Matrix<float, 3, 1> gravity(0.0f, 0.0f, -_G_GRAVITY); // Z-Up Earth Frame
    Matrix<float, 3, 1> acc_world = R * acc_clean + gravity;
    
    _pos += _vel * dt + 0.5f * acc_world * dt * dt;
    _vel += acc_world * dt;

    // 3. Form Process Jacobian (_F)
    _G.block<3, 3>(6, 6) = -skewSymmetric(gyro_clean);      // dtheta_dot / dtheta
    _G.block<3, 3>(3, 6) = -R * skewSymmetric(acc_clean);   // dv_dot / dtheta
    _G.block<3, 3>(3, 9) = -R;                              // dv_dot / dab (Both in physical units)

    _F = _I15 + _G * dt;

    // 4. Propagate Covariance
    compute_process_covariance(dt);
    _cov = _F * _cov * _F.transpose() + _Q;
}

void NavMEKF::updateBaro(float measuredAltitude, float R_baro) {
    // Barometer maps to Z-Position [Index 2]
    Matrix<float, 1, 15> H = Matrix<float, 1, 15>::Zero();
    H(0, 2) = 1.0f; 

    float S = (H * _cov * H.transpose())(0, 0) + R_baro;
    _K15x1 = _cov * H.transpose() * (1.0f / S);

    float innovation = measuredAltitude - _pos(2);
    injectErrorState(_K15x1 * innovation);
    
    _temp15x15 = _I15 - _K15x1 * H;
    _cov = _temp15x15 * _cov * _temp15x15.transpose() + _K15x1 * R_baro * _K15x1.transpose(); // Joseph Form
}

void NavMEKF::updateMag(const Matrix<float, 3, 1>& mag_meas, const Matrix<float, 3, 1>& mag_ref, float R_mag) {
    if (mag_meas.squaredNorm() < 1e-6f) return;

    Matrix<float, 3, 3> R = quatToMatrix(_quat);
    
    // 1. Project Body-Mag into World Frame to find Measured Heading
    Matrix<float, 3, 1> m_world = R * mag_meas.normalized();
    float psi_meas = atan2f(m_world(1), m_world(0));
    float psi_ref  = atan2f(mag_ref(1), mag_ref(0));

    // 2. Innovation (Yaw Error)
    float innovation = psi_meas - psi_ref;
    while (innovation >  3.14159265f) innovation -= 6.28318531f; 
    while (innovation < -3.14159265f) innovation += 6.28318531f; 

    // 3. Construct Jacobian (H)
    // We observe the rotation error around the World-Z axis.
    // Mapping world-Z error into body error coordinates: h_body = R.row(2).
    Matrix<float, 1, 15> H = Matrix<float, 1, 15>::Zero();
    H(0, 6) = R(2, 0); 
    H(0, 7) = R(2, 1);
    H(0, 8) = R(2, 2);

    // 4. Kalman Gain Calculation (1D Update)
    float S = (H * _cov * H.transpose())(0, 0) + R_mag;
    _K15x1 = _cov * H.transpose() / S;

    // 5. Update State
    injectErrorState(_K15x1 * innovation);

    // 6. Update Covariance
    _temp15x15 = _I15 - _K15x1 * H;
    _cov = _temp15x15 * _cov * _temp15x15.transpose() + _K15x1 * R_mag * _K15x1.transpose();
}
void NavMEKF::updateAccel(const Matrix<float, 3, 1>& acc_meas, const Matrix<float, 3, 3>& R_acc) {
    // Expected gravity in body frame: R^T * [0, 0, g]
    Matrix<float, 3, 1> world_gravity(0.0f, 0.0f, _G_GRAVITY);
    Matrix<float, 3, 1> g_body = rotateInverse(_quat, world_gravity);
    Matrix<float, 3, 1> expected_acc = g_body + _accel_bias;

    // H maps to Attitude Error [6:8] and Accel Bias [9:11]
    Matrix<float, 3, 15> H = Matrix<float, 3, 15>::Zero();
    H.block<3, 3>(0, 6) = skewSymmetric(g_body); 
    H.block<3, 3>(0, 9) = Matrix<float, 3, 3>::Identity();

    Matrix<float, 3, 3> S = H * _cov * H.transpose() + R_acc;
    _K15x3 = _cov * H.transpose() * S.inverse();

    // Measurement is in m/s^2 for physical bias tracking
    Matrix<float, 3, 1> innovation = acc_meas - expected_acc;
    injectErrorState(_K15x3 * innovation);

    _temp15x15 = _I15 - _K15x3 * H;
    _cov = _temp15x15 * _cov * _temp15x15.transpose() + _K15x3 * R_acc * _K15x3.transpose();
}

void NavMEKF::updateVelocity(const Matrix<float, 3, 1>& vel_meas, const Matrix<float, 3, 3>& R_vel) {
    // Velocity maps to Velocity Error [3:6]
    Matrix<float, 3, 15> H = Matrix<float, 3, 15>::Zero();
    H.block<3, 3>(0, 3) = Matrix<float, 3, 3>::Identity();

    Matrix<float, 3, 3> S = H * _cov * H.transpose() + R_vel;
    _K15x3 = _cov * H.transpose() * S.inverse();

    Matrix<float, 3, 1> innovation = vel_meas - _vel;
    injectErrorState(_K15x3 * innovation);

    _temp15x15 = _I15 - _K15x3 * H;
    _cov = _temp15x15 * _cov * _temp15x15.transpose() + _K15x3 * R_vel * _K15x3.transpose();
}

void NavMEKF::updateGPS(const Matrix<float, 3, 1>& pos_meas, const Matrix<float, 3, 1>& vel_meas, const Matrix<float, 6, 6>& R_gps) {
    // GPS maps to Position [0:3] and Velocity [3:6]
    Matrix<float, 6, 15> H = Matrix<float, 6, 15>::Zero();
    H.block<3, 3>(0, 0) = Matrix<float, 3, 3>::Identity();
    H.block<3, 3>(3, 3) = Matrix<float, 3, 3>::Identity();

    Matrix<float, 6, 6> S = H * _cov * H.transpose() + R_gps;
    _K15x6 = _cov * H.transpose() * S.inverse();

    Matrix<float, 6, 1> innovation;
    innovation << (pos_meas - _pos), (vel_meas - _vel);
    
    injectErrorState(_K15x6 * innovation);
    
    _temp15x15 = _I15 - _K15x6 * H;
    _cov = _temp15x15 * _cov * _temp15x15.transpose() + _K15x6 * R_gps * _K15x6.transpose();
}

void NavMEKF::injectErrorState(const Matrix<float, 15, 1>& error_state) {
    // 1. Additive updates
    _pos += error_state.segment<3>(0);
    _vel += error_state.segment<3>(3);
    _accel_bias += error_state.segment<3>(9);
    _gyro_bias += error_state.segment<3>(12);

    // 2. The Multiplicative Update for Attitude
    Matrix<float, 3, 1> attitude_error = error_state.segment<3>(6);
    Matrix<float, 4, 1> dq;
    dq << 1.0f, 0.5f * attitude_error(0), 0.5f * attitude_error(1), 0.5f * attitude_error(2);
    
    _quat = quatMultiply(_quat, dq);
    _quat.normalize();
}

// --- Mathematical Helpers ---

Matrix<float, 3, 3> NavMEKF::skewSymmetric(const Matrix<float, 3, 1>& v) const {
    Matrix<float, 3, 3> S;
    S <<  0.0f, -v(2),  v(1),
          v(2),  0.0f, -v(0),
         -v(1),  v(0),  0.0f;
    return S;
}

Matrix<float, 4, 1> NavMEKF::quatMultiply(const Matrix<float, 4, 1>& q1, const Matrix<float, 4, 1>& q2) const {
    Matrix<float, 4, 1> r;
    r(0) = q1(0)*q2(0) - q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3);
    r(1) = q1(0)*q2(1) + q1(1)*q2(0) + q1(2)*q2(3) - q1(3)*q2(2);
    r(2) = q1(0)*q2(2) - q1(1)*q2(3) + q1(2)*q2(0) + q1(3)*q2(1);
    r(3) = q1(0)*q2(3) + q1(1)*q2(2) - q1(2)*q2(1) + q1(3)*q2(0);
    return r;
}

Matrix<float, 3, 3> NavMEKF::quatToMatrix(const Matrix<float, 4, 1>& q) const {
    float qw = q(0), qx = q(1), qy = q(2), qz = q(3);
    Matrix<float, 3, 3> R;
    R << 1.0f - 2.0f*(qy*qy + qz*qz),  2.0f*(qx*qy - qz*qw),        2.0f*(qx*qz + qy*qw),
         2.0f*(qx*qy + qz*qw),         1.0f - 2.0f*(qx*qx + qz*qz), 2.0f*(qy*qz - qx*qw),
         2.0f*(qx*qz - qy*qw),         2.0f*(qy*qz + qx*qw),        1.0f - 2.0f*(qx*qx + qy*qy);
    return R;
}

Matrix<float, 3, 1> NavMEKF::rotateInverse(const Matrix<float, 4, 1>& q, const Matrix<float, 3, 1>& v) const {
    Matrix<float, 3, 3> R = quatToMatrix(q);
    return R.transpose() * v;
}
