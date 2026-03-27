#include "AttitudeEstimator.h"
#include "Config_voo.h"
#include <math.h>
#include "MPU9250_HAL.h"

/**
 * @brief Initialize the estimator with an IMU pointer.
 * @param imu Pointer to an IMUSensor instance.
 */
AttitudeEstimator::AttitudeEstimator(IMUSensor* imu) : _imu(imu) {
    resetOrientation(); 
}

/**
 * @brief Main update loop. Pulls raw data from IMU and updates the active filter.
 * @param dt Time delta in seconds.
 */
void AttitudeEstimator::update(float dt, bool ignoreAccel, bool physicalZAxisDown) {
    if (!_imu) return; 
    _deltaT = dt;

    float ax = _imu->getAccX();
    float ay = _imu->getAccY();
    float az = _imu->getAccZ();
    
    // [TRANSFORM] If physicalZAxisDown is true, it means the sensor is mounted 
    // upside down relative to the rocket. We apply a 180-X flip to bring it 
    // into the internal Z-Up frame (+1G gravity) that the filter expects.
    if (physicalZAxisDown) {
        // Rotation of 180 degrees around X: [x, -y, -z]
        ay = -ay; az = -az;
    }

    _transformedAccX = ax;
    _transformedAccY = ay;
    _transformedAccZ = az;

    // Automatic Accel Masking: Ignore if beyond thresholds
    float a_norm = sqrt(ax * ax + ay * ay + az * az);
    if (a_norm > ORIENTATION_MASK_MAX_G || a_norm < ORIENTATION_MASK_MIN_G) {
        ignoreAccel = true;
    }

    float gx_dps = _imu->getGyroX_rads() * RAD_TO_DEG;
    float gy_dps = _imu->getGyroY_rads() * RAD_TO_DEG;
    float gz_dps = _imu->getGyroZ_rads() * RAD_TO_DEG;

    if (physicalZAxisDown) {
        // Rotation of 180 degrees around X: [x, -y, -z]
        gy_dps = -gy_dps; gz_dps = -gz_dps;
    }

    // [DIAGNOSTIC] Store transformed gyro values
    _transformedGyroX = gx_dps;
    _transformedGyroY = gy_dps;
    _transformedGyroZ = gz_dps;

    // [FIX] Apply Gyro Cutoff (Deadband) to ignore stationary vibration/noise
    float cutoff_dps = ATTITUDE_GYRO_CUTOFF_DPS; 
    if (abs(gx_dps) < cutoff_dps) gx_dps = 0.0f;
    if (abs(gy_dps) < cutoff_dps) gy_dps = 0.0f;
    if (abs(gz_dps) < cutoff_dps) gz_dps = 0.0f;
    
    // Convert back to rad/s for the filter
    float gx = gx_dps * DEG_TO_RAD;
    float gy = gy_dps * DEG_TO_RAD;
    float gz = gz_dps * DEG_TO_RAD;

    float mx = 0.0f;
    float my = 0.0f;
    float mz = 0.0f;

    if (_useMagnetometer) {
        // [AXIS ALIGNMENT FIX] 
        // The MPU9250's internal AK8963 Magnetometer is physically rotated relative to the MPU6500 Accel/Gyro:
        // AK8963_X aligns with MPU_Y, AK8963_Y aligns with MPU_X, AK8963_Z aligns with MPU_-Z.
        float rawMagX = _imu->getMagX(); 
        float rawMagY = _imu->getMagY(); 
        float rawMagZ = _imu->getMagZ(); 

        mx = rawMagY;   // Align Mag Y to Accel X (Right)
        my = rawMagX;   // Align Mag X to Accel Y (Forward)
        mz = rawMagZ;   // Already flipped in HAL to be Up-positive

        if (physicalZAxisDown) {
            // Rotation of 180 degrees around X: [x, -y, -z]
            my = -my; mz = -mz;
        }
    }

    MPU9250_HAL* mpuHal = (MPU9250_HAL*)_imu;

    switch (_filterSel) {
        case AttitudeFilterSel::MADGWICK:
            // Always run 9-axis fusion if Mag is enabled.
            updateMadgwick(ignoreAccel ? 0 : ax, ignoreAccel ? 0 : ay, ignoreAccel ? 0 : az, gx, gy, gz, mx, my, mz);
            break;
        case AttitudeFilterSel::MAHONY:
            updateMahony(ignoreAccel ? 0 : ax, ignoreAccel ? 0 : ay, ignoreAccel ? 0 : az, gx, gy, gz, mx, my, mz);
            break;
        case AttitudeFilterSel::NONE:
            updateNone(ax, ay, az, gx, gy, gz);
            break;
        case AttitudeFilterSel::EKF:
            // Placeholder: Not implemented
            break;
    }
}

/**
 * @brief Select which orientation filter to use.
 * @param sel Enum value for NONE, MADGWICK, or MAHONY.
 */
void AttitudeEstimator::selectFilter(AttitudeFilterSel sel) {
    _filterSel = sel;
}

/**
 * @brief Enable or disable the zeta-based drift learning for Madgwick filter.
 * @param enabled True to enable.
 */
void AttitudeEstimator::setDriftLearning(bool enabled) {
    _useZeta = enabled;
}

/**
 * @brief Set the filter's beta parameter.
 * @param errorDegPerSec Estimated gyroscope drift in degrees per second.
 */
void AttitudeEstimator::setFilterBeta(float errorDegPerSec) {
    float gyroErr = PI * (errorDegPerSec / 180.0f);
    _beta = sqrt(3.0f / 4.0f) * gyroErr;
    _zeta = _beta; // [FIX] Synchronize Integral Gain to proportional bounds to prevent Windup
}

/**
 * @brief Resets the internal orientation quaternion.
 * @param physicalZAxisDown If true, starts flipped 180deg (Z pointing down).
 */
void AttitudeEstimator::resetOrientation(bool unused) {
    _q[0] = 1.0f; _q[1] = 0.0f; _q[2] = 0.0f; _q[3] = 0.0f;
}

void AttitudeEstimator::resetEstimatorState() {
    _q[0] = 1.0f; _q[1] = 0.0f; _q[2] = 0.0f; _q[3] = 0.0f;
    _w_bx = 0.0f; _w_by = 0.0f; _w_bz = 0.0f;
    _ix = 0.0f; _iy = 0.0f; _iz = 0.0f;
}

// --- Euler Angle Computations ---
float AttitudeEstimator::getRoll()  const { return computeRoll(); }
float AttitudeEstimator::getPitch() const { return computePitch();}
float AttitudeEstimator::getYaw()   const { return computeYaw();  }

/**
 * @brief Computes Roll from current quaternion.
 * @return Roll in degrees.
 */
float AttitudeEstimator::computeRoll() const {
    float roll = atan2f(2.0f * (_q[0] * _q[1] + _q[2] * _q[3]), 1.0f - 2.0f * (_q[1] * _q[1] + _q[2] * _q[2]));
    return roll * RAD_TO_DEG;
}

/**
 * @brief Computes Pitch from current quaternion.
 * @return Pitch in degrees.
 */
float AttitudeEstimator::computePitch() const {
    float sinp = 2.0f * (_q[0] * _q[2] - _q[3] * _q[1]);
    if (abs(sinp) >= 1) {
        return copysignf(M_PI / 2.0f, sinp) * RAD_TO_DEG; 
    } else {
        return asinf(sinp) * RAD_TO_DEG;
    }
}

/**
 * @brief Computes Yaw from current quaternion.
 * @return Yaw in degrees.
 */
float AttitudeEstimator::computeYaw() const {
    float yaw = atan2f(2.0f * (_q[0] * _q[3] + _q[1] * _q[2]), 1.0f - 2.0f * (_q[2] * _q[2] + _q[3] * _q[3]));
    return yaw * RAD_TO_DEG;
}

// --- Advanced Math Getters ---

/**
 * @brief Calculates the tilt angle relative to the earth's gravity vector.
 * @param physicalZAxisDown True if the IMU is mounted Z-axis pointing down.
 * @return Tilt angle in degrees [0-180].
 */
float AttitudeEstimator::getTilt(bool unused) const {
    float qw = _q[0];
    float qx = _q[1];
    float qy = _q[2];
    float qz = _q[3];

    // Standard Z-Up "upward" vector projected into body: [2(q1q3 - q0q2), 2(q0q1 + q2q3), q0^2 - q1^2 - q2^2 + q3^2]
    // The Z-component is cos(tilt).
    float cos_tilt = qw * qw - qx * qx - qy * qy + qz * qz;
    if (cos_tilt > 1.0f) cos_tilt = 1.0f;
    else if (cos_tilt < -1.0f) cos_tilt = -1.0f;
    
    return acosf(cos_tilt) * RAD_TO_DEG;
}

/**
 * @brief Computes Z-axis acceleration in the world frame with gravity removed.
 * @details Uses the current quaternion to rotate the body-frame acceleration 
 *          to the world frame. Note: On pad, this should be close to 0.0.
 * @return Vertical acceleration in m/s^2.
 */
float AttitudeEstimator::getNetVerticalAcceleration(bool isZDown) const {
    float qw = _q[0]; float qx = _q[1]; float qy = _q[2]; float qz = _q[3];

    // Project the ALREADY TRANSFORMED body acceleration into the world frame.
    // _transformedAccX/Y/Z are updated in update() and already account for mount orientation.
    float worldZAcceleration =
        2.0f * (qx * qz - qw * qy) * _transformedAccX +
        2.0f * (qw * qx + qy * qz) * _transformedAccY +
        (qw * qw - qx * qx - qy * qy + qz * qz) * _transformedAccZ;

    // the Z-Up transformation in update() already handles the mounting.
    // worldZ should be ~ +1.0G on the pad regardless of PHYSICAL_Z_AXIS_DOWN.
    float netG = (worldZAcceleration - 1.0f);

    return netG * _G_GRAVITY;
}

/**
 * @brief Pure angular rate integration (no correction).
 */
void AttitudeEstimator::updateNone(float ax, float ay, float az, float gx, float gy, float gz) {
    float q0 = _q[0], q1 = _q[1], q2 = _q[2], q3 = _q[3];
    _q[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * _deltaT;
    _q[1] += 0.5f * (q0 * gx + q2 * gz - q3 * gy) * _deltaT;
    _q[2] += 0.5f * (q0 * gy - q1 * gz + q3 * gx) * _deltaT;
    _q[3] += 0.5f * (q0 * gz + q1 * gy - q2 * gx) * _deltaT;
    
    float recipNorm = 1.0f / sqrt(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
    _q[0] *= recipNorm; _q[1] *= recipNorm; _q[2] *= recipNorm; _q[3] *= recipNorm;
}

/**
 * @brief Madgwick orientation filter implementation.
 * @details Ported from the original MPU9250 package. Supports magnetometer if valid.
 */
void AttitudeEstimator::updateMadgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    double q0 = _q[0], q1 = _q[1], q2 = _q[2], q3 = _q[3];
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double hx, hy;
    double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    double a_norm_sq = ax * ax + ay * ay + az * az;
    double m_norm_sq = mx * mx + my * my + mz * mz;

    s0 = 0; s1 = 0; s2 = 0; s3 = 0;

    // Common term pre-computation
    _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
    q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
    q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
    q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;

    // 1. Accelerometer Component
    if (a_norm_sq > 0.0f) {
        recipNorm = 1.0 / sqrt(a_norm_sq);
        ax *= (float)recipNorm; ay *= (float)recipNorm; az *= (float)recipNorm;

        s0 += -_2q2 * (2.0f * (q1q3 - q0q2) - ax) + _2q1 * (2.0f * (q0q1 + q2q3) - ay);
        s1 += _2q3 * (2.0f * (q1q3 - q0q2) - ax) + _2q0 * (2.0f * (q0q1 + q2q3) - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az);
        s2 += -_2q0 * (2.0f * (q1q3 - q0q2) - ax) + _2q3 * (2.0f * (q0q1 + q2q3) - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az);
        s3 += _2q1 * (2.0f * (q1q3 - q0q2) - ax) + _2q2 * (2.0f * (q0q1 + q2q3) - ay);
    }

    // 2. Magnetometer Component
    if (m_norm_sq > 0.0f) {
        recipNorm = 1.0 / sqrt(m_norm_sq);
        mx *= (float)recipNorm; my *= (float)recipNorm; mz *= (float)recipNorm;

        _2q0mx = 2.0f * q0 * mx; _2q0my = 2.0f * q0 * my; _2q0mz = 2.0f * q0 * mz; _2q1mx = 2.0f * q1 * mx;
        _2q0q2 = 2.0f * q0 * q2; _2q2q3 = 2.0f * q2 * q3;

        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx; _4bz = 2.0f * _2bz;

        s0 += _magWeight * (-_2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (0.5f - q1q1 - q2q2) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz));
        s1 += _magWeight * (_2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz));
        s2 += _magWeight * ((-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (0.5f - q1q1 - q2q2) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz));
        s3 += _magWeight * ((-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (0.5f - q1q1 - q2q2) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz));
    }

    double s_norm_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
    if (s_norm_sq > 0.0) {
        recipNorm = 1.0 / sqrt(s_norm_sq);
        s0 *= (float)recipNorm; s1 *= (float)recipNorm; s2 *= (float)recipNorm; s3 *= (float)recipNorm;
    }

    if (_useZeta) {
        double s_w_x = 2.0f * (q0 * s1 - q1 * s0 - q2 * s3 + q3 * s2);
        double s_w_y = 2.0f * (q0 * s2 + q1 * s3 - q2 * s0 - q3 * s1);
        double s_w_z = 2.0f * (q0 * s3 - q1 * s2 + q2 * s1 - q3 * s0);

        _w_bx += _zeta * s_w_x * _deltaT;
        _w_by += _zeta * s_w_y * _deltaT;
        _w_bz += _zeta * s_w_z * _deltaT;
    }

    gx -= _w_bx; gy -= _w_by; gz -= _w_bz; 

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - _beta * s0;
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - _beta * s1;
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - _beta * s2;
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - _beta * s3;

    q0 += qDot1 * _deltaT; q1 += qDot2 * _deltaT; q2 += qDot3 * _deltaT; q3 += qDot4 * _deltaT;

    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    _q[0] = q0 * recipNorm; _q[1] = q1 * recipNorm; _q[2] = q2 * recipNorm; _q[3] = q3 * recipNorm;
}

/**
 * @brief Mahony orientation filter implementation.
 * @details Uses a Proportional-Integral feedback loop to correct gyroscope drift using accel/mag.
 */
void AttitudeEstimator::updateMahony(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;  
    float qa, qb, qc;
    float tmp;

    tmp = ax * ax + ay * ay + az * az;
    if (tmp > 0.0f) {
        recipNorm = 1.0f / sqrt(tmp);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        vx = _q[1] * _q[3] - _q[0] * _q[2];
        vy = _q[0] * _q[1] + _q[2] * _q[3];
        vz = _q[0] * _q[0] - 0.5f + _q[3] * _q[3];

        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        double m_norm = mx * mx + my * my + mz * mz;
        if (m_norm > 0.0) {
            recipNorm = 1.0 / sqrt(m_norm);
            mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

            float hx = mx * _q[0] * _q[0] - 2.0f * _q[0] * my * _q[3] + 2.0f * _q[0] * mz * _q[2] + mx * _q[1] * _q[1] + 2.0f * _q[1] * my * _q[2] + 2.0f * _q[1] * mz * _q[3] - mx * _q[2] * _q[2] - mx * _q[3] * _q[3];
            float hy = 2.0f * _q[0] * mx * _q[3] + my * _q[0] * _q[0] - 2.0f * _q[0] * mz * _q[1] + 2.0f * _q[1] * mx * _q[2] - my * _q[1] * _q[1] + my * _q[2] * _q[2] + 2.0f * _q[2] * mz * _q[3] - my * _q[3] * _q[3];
            float bx = sqrt(hx * hx + hy * hy);
            float bz = -2.0f * _q[0] * mx * _q[2] + 2.0f * _q[0] * my * _q[1] + mz * _q[0] * _q[0] + 2.0f * _q[1] * mx * _q[3] - mz * _q[1] * _q[1] + 2.0f * _q[2] * my * _q[3] - mz * _q[2] * _q[2] + mz * _q[3] * _q[3];

            float wx = 2.0f * bx * (0.5f - _q[2] * _q[2] - _q[3] * _q[3]) + 2.0f * bz * (_q[1] * _q[3] - _q[0] * _q[2]);
            float wy = 2.0f * bx * (_q[1] * _q[2] - _q[0] * _q[3]) + 2.0f * bz * (_q[0] * _q[1] + _q[2] * _q[3]);
            float wz = 2.0f * bx * (_q[0] * _q[2] + _q[1] * _q[3]) + 2.0f * bz * (0.5f - _q[1] * _q[1] - _q[2] * _q[2]);

            ex += _magWeight * (my * wz - mz * wy);
            ey += _magWeight * (mz * wx - mx * wz);
            ez += _magWeight * (mx * wy - my * wx);
        }

        if (_Ki > 0.0f) {
            _ix += _Ki * ex * _deltaT;
            _iy += _Ki * ey * _deltaT;
            _iz += _Ki * ez * _deltaT;
            gx += _ix; gy += _iy; gz += _iz;
        }

        gx += _Kp * ex; gy += _Kp * ey; gz += _Kp * ez;
    }

    float halfDt = 0.5f * _deltaT;
    gx *= halfDt; gy *= halfDt; gz *= halfDt;
    qa = _q[0]; qb = _q[1]; qc = _q[2];
    _q[0] += (-qb * gx - qc * gy - _q[3] * gz);
    _q[1] += (qa * gx + qc * gz - _q[3] * gy);
    _q[2] += (qa * gy - qb * gz + _q[3] * gx);
    _q[3] += (qa * gz + qb * gy - qc * gx);

    recipNorm = 1.0f / sqrt(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
    _q[0] *= recipNorm; _q[1] *= recipNorm; _q[2] *= recipNorm; _q[3] *= recipNorm;
}

/**
 * @brief Placeholder for EKF (Extended Kalman Filter) update.
 */
void AttitudeEstimator::updateEKF(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    // TODO: Implement EKF update
}

void AttitudeEstimator::setMagnetometerWeight(float weight) {
    if (weight < 0.0f) weight = 0.0f;
    if (weight > 1.0f) weight = 1.0f;
    _magWeight = weight;
}

/**
 * @brief Enable or disable magnetometer fusion.
 * @param use True to enable.
 */
void AttitudeEstimator::setUseMagnetometer(bool use) {
    _useMagnetometer = use;
}
