#include "KalmanFilter.hh" // Ajuste o caminho se necessário
#include "Config_voo.h" 
#include <ArduinoEigenDense.h>

using namespace Eigen;

KalmanFilter::KalmanFilter() : k(0) {}
KalmanFilter::~KalmanFilter() {}

/**
 * @brief Initializes the Kalman Filter matrices and initial states.
 * @param F_in State Transition Matrix.
 * @param G_in Control Matrix.
 * @param H_in Measurement Matrix.
 * @param Q_in Process Noise Covariance.
 * @param R_in Measurement Noise Covariance (Initial).
 * @param P0_in Initial Error Covariance.
 * @param X0_in Initial State Estimate.
 */
void KalmanFilter::init(const Matrix<float, KF_STATES, KF_STATES>& F_in,
                        const Matrix<float, KF_STATES, KF_INPUTS>& G_in,
                        const Matrix<float, KF_MEASUREMENTS, KF_STATES>& H_in, 
                        const Matrix<float, KF_STATES, KF_STATES>& Q_in, 
                        const Matrix<float, KF_MEASUREMENTS, KF_MEASUREMENTS>& R_in, 
                        const Matrix<float, KF_STATES, KF_STATES>& P0_in, 
                        const Matrix<float, KF_STATES, 1>& X0_in) {
    F = F_in; G = G_in; H = H_in; Q = Q_in; R = R_in; P0 = P0_in; X0 = X0_in;
    
    estimatesAposteriori = X0;
    covarianceAposteriori = P0;
    k = 0;
}

/**
 * @brief Performs the Prediction (Time Update) step of the Kalman Filter.
 * @details This step projects the current state estimate and error covariance forward in time
 * using the system's dynamic model (Newton's laws).
 * * Equations:
 * 1. State Extrapolation:  x(k|k-1) = F * x(k-1|k-1) + G * u(k)
 * (New Position = Old Position + Velocity*dt + 0.5*acc*dt^2)
 * 2. Covariance Extrapolation: P(k|k-1) = F * P(k-1|k-1) * F' + Q
 * (Increases uncertainty P because prediction is never perfect, adding process noise Q)
 * * @param U Control input vector (Net Vertical Acceleration from IMU in m/s^2).
 */
void KalmanFilter::Predict(const Matrix<float, KF_INPUTS, 1>& U) {

    estimatesApriori = F * estimatesAposteriori + G * U;
    covarianceApriori = F * covarianceAposteriori * F.transpose() + Q;
    k++;
}

/**
 * @brief Performs the Update (Measurement Correction) step of the Kalman Filter.
 * @details This step corrects the predicted state using the actual measurement from the sensors.
 * It calculates the optimal Kalman Gain (K) to weigh the trust between the model and the sensor.
 * * Equations:
 * 1. Innovation Covariance: S = H * P * H' + R
 * 2. Kalman Gain:           K = P * H' * S^-1
 * 3. Innovation (Residual): y = z - H * x
 * 4. State Update:          x = x + K * y
 * 5. Covariance Update:     P = (I - K * H) * P
 * * @param Z Measurement vector (Barometric Altitude in meters).
 * @param R Measurement noise covariance matrix (allows adapting sensor trust dynamically).
 */
void KalmanFilter::Update(const Matrix<float, KF_MEASUREMENTS, 1>& Z, 
                          const Matrix<float, KF_MEASUREMENTS, KF_MEASUREMENTS>& R) {
    
    Matrix<float, KF_MEASUREMENTS, KF_MEASUREMENTS> Sk = H * covarianceApriori * H.transpose() + R;
    KalmanGain = covarianceApriori * H.transpose() * Sk.inverse();
    errors = Z - H * estimatesApriori;
    estimatesAposteriori = estimatesApriori + KalmanGain * errors;
    Matrix<float, KF_STATES, KF_STATES> In = Matrix<float, KF_STATES, KF_STATES>::Identity();
    covarianceAposteriori = (In - KalmanGain * H) * covarianceApriori;
}

/**
 * @brief Gets the current best estimate of the state.
 * @return Vector containing [Altitude (m), Vertical Velocity (m/s)].
 */
Matrix<float, KF_STATES, 1> KalmanFilter::getPosterioriState() {
    return estimatesAposteriori;
}