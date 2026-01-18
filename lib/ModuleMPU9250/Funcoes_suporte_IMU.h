#ifndef FUNCOES_SUPORTE_IMU_H  
#define FUNCOES_SUPORTE_IMU_H

// Dentro de Funcoes_suporte_IMU.h
#include "MPU9250.h" 

extern MPU9250 mpu;
extern MPU9250Setting mpuConfig;

void saveCalibration(bool print);
void eraseCalibration();
void loadCalibration(bool print);
bool hasCalibrationDataIMU();
void print_vector(const char* label, const float values[], float scale = 1.0f, int decimals = 2,const char* unit_label = nullptr); 
void calibration_IMU(bool print);   
bool setup_IMU(bool calibrar_se_necessario, bool perform_fine_tuning_on_calib, bool print_params);
void print_roll_pitch_yaw();
float calcTilt();
bool adjustCalibrationIteratively(int samples_per_iteration, bool print_debug,float accel_tolerance_g, float gyro_tolerance_dps, int max_iterations);
void test_bias_for_adjust(int samples, float result_accel_g[3], float result_gyro_dps[3]);
void write_byte_local(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t read_byte_local(uint8_t address, uint8_t subAddress);
float computeNetAcceleration(bool print_debug = false, float ax_input = 0.0f, float ay_input = 0.0f, float az_input = 0.0f, bool autoUpdate = true);
void test_fine_tuning();
void reset_orientation(bool z_axis_down);
void setDriftLearning(bool enabled);
void setFilterBeta(float errorDegPerSec);

#endif  // FUNCOES_SUPORTE_IMU_H