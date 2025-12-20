#ifndef KALMANFILTER_HH
#define KALMANFILTER_HH

#include <ArduinoEigenDense.h>


// Definindo os tamanhos do filtro como constantes para clareza
const int KF_STATES = 2;       // [altitude, velocidade]
const int KF_INPUTS = 1;       // [aceleracao_liquida]
const int KF_MEASUREMENTS = 2; // [altitude_medida, velocidade_medida]

using namespace Eigen;

class KalmanFilter {
public:
    KalmanFilter();
    ~KalmanFilter();

    void init(const Matrix<float, KF_STATES, KF_STATES>& F_in,
              const Matrix<float, KF_STATES, KF_INPUTS>& G_in,
              const Matrix<float, KF_MEASUREMENTS, KF_STATES>& H_in, 
              const Matrix<float, KF_STATES, KF_STATES>& Q_in, 
              const Matrix<float, KF_MEASUREMENTS, KF_MEASUREMENTS>& R_in, 
              const Matrix<float, KF_STATES, KF_STATES>& P0_in, 
              const Matrix<float, KF_STATES, 1>& X0_in);

    void Predict(const Matrix<float, KF_INPUTS, 1>& U);
    void Update(const Matrix<float, KF_MEASUREMENTS, 1>& Z, 
                const Matrix<float, KF_MEASUREMENTS, KF_MEASUREMENTS>& R);
    
    Matrix<float, KF_STATES, 1> getPosterioriState();

private:
    unsigned int k;

    Matrix<float, KF_STATES, KF_STATES>       F, Q, P0, covarianceAposteriori, covarianceApriori;
    Matrix<float, KF_STATES, KF_INPUTS>        G;
    Matrix<float, KF_MEASUREMENTS, KF_STATES>  H;
    Matrix<float, KF_MEASUREMENTS, KF_MEASUREMENTS> R;
    Matrix<float, KF_STATES, 1>                X0, estimatesAposteriori, estimatesApriori;
    Matrix<float, KF_STATES, KF_MEASUREMENTS>  KalmanGain;
    Matrix<float, KF_MEASUREMENTS, 1>          errors;
};

#endif