# Future Implementations and Improvements

This document tracks planned features, enhancements, and technical debt for the ESP-AIRBRAKE project.

## Planned Improvements

- [ ] **State Estimation (EKF)**: 
    - Implement an **Extended Kalman Filter (EKF)** or Unscented Kalman Filter (UKF) to replace the current basic IMU/Baro fusion.
    - Improve handling of nonlinear dynamics (e.g., orientation tracking during high-G acceleration and airbrake deployment).
- [ ] **Enhanced HIL Simulation**: 
    - Improve Hardware-in-the-Loop (HIL) simulation accuracy.
    - Implement timing simulations for sensor readings to better mimic real-world acquisition delays, jitter, and noise profiles.
- [X] **Filesystem Migration**: 
    - Move from FFat to **LittleFS** for better reliability, wear leveling, and power-fail safety.
- [ ] **Advanced Apogee Control**:
    - Evolve the airbrake deployment algorithm from classical PID to **Model Predictive Control (MPC)** or **LQR**, leveraging the existing `DragTable` and `AltitudeSpeedTable` for optimal trajectory tracking.
- [ ] **Servo Trajectory Generation**:
    - Replace instant position commands with smoothed, jerk-limited trajectories to reduce power spikes, prevent stripped gears, and improve aerodynamic stability during deployment.
    - **ZUKF Bias Estimation**: Automatically estimate gyroscope and accelerometer bias drift during pre-flight.
- [ ] **Elite Logging Optimizations**:
    - **SD Card Pre-allocation**: Implement contiguous file pre-allocation to eliminate FAT32 fragmentation stalls during high-speed writes.
    - **Full Binary SD Logging**: Move from CSV to 100% binary logging on SD to eliminate `printf` overhead and fixed-width formatting.
    - **CRC-16/32 Integrity**: Add checksums to every record to ensure zero data corruption during high-vibration flight phases.
    - **Static DMA Buffers**: Replace `String` buffers with 4-byte aligned static memory arrays to maximize the efficiency of the ESP32-S3's SDIO-DMA engine.
