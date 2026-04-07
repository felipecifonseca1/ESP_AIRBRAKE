# Future Implementations and Improvements

This document tracks planned features, enhancements, and technical debt for the ESP-AIRBRAKE project.

## Planned Improvements

- [x] **State Estimation (NavMEKF)**: 
    - **Status**: Implemented high-performance 15-state Navigation Multiplicative Extended Kalman Filter (NavMEKF).
    - Handle nonlinear dynamics during high-G acceleration and airbrake deployment.
- [ ] **Enhanced HIL Simulation**: 
    - Improve Hardware-in-the-Loop (HIL) simulation accuracy.
    - Implement timing simulations for sensor readings to better mimic real-world acquisition delays, jitter, and noise profiles.
- [X] **Filesystem Migration**: 
    - Move from FFat to **LittleFS** for better reliability, wear leveling, and power-fail safety.
- [ ] **Hardware Redundancy & Reliability**:
    - **Dual IMU Support**: Migrate MPU9250 to **SPI** (10MHz) and add a secondary redundant IMU with a voting/consensus scheme.
    - **Dual Barometers**: Implement asynchronous staggered triggering (e.g., 200Hz effective resolution) using two BMP280s on separate I2C buses.
    - **Non-blocking Sensor Acquisition**: Move from blocking I2C polling to a state-machine based "Trigger -> Process -> Collect" model to reclaim 2-4ms of CPU time per loop.
- [ ] **Advanced Apogee Control**:
    - Evolve the airbrake deployment algorithm from classical PID to **Model Predictive Control (MPC)** or **LQR**, leveraging the existing `DragTable` and `AltitudeSpeedTable` for optimal trajectory tracking.
- [ ] **Servo Trajectory Generation**:
    - Replace instant position commands with smoothed, jerk-limited trajectories to reduce power spikes, prevent stripped gears, and improve aerodynamic stability during deployment.
- [x] **State Extension**:
    - Implement real-time **Gyro and Accelerometer bias estimation** (Now part of the 15-state NavMEKF).
- [ ] **Elite Logging Optimizations**:
    - **SD Card Pre-allocation**: Implement contiguous file pre-allocation to eliminate FAT32 fragmentation stalls during high-speed writes.
    - **Full Binary SD Logging**: Move from CSV to 100% binary logging on SD to eliminate `printf` overhead and fixed-width formatting.
    - **3rd State in KF (Acceleration bias)**: Add acceleration bias to the state estimation.
   
