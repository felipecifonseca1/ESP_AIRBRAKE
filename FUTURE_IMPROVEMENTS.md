# Future Implementations and Improvements

This document tracks planned features, enhancements, and technical debt for the ESP-AIRBRAKE project.

## Planned Improvements

- [ ] **3rd State in KF (Acceleration bias)**: Add acceleration bias to the state estimation.
- [ ] **Enhanced HIL Simulation**: 
    - Improve Hardware-in-the-Loop (HIL) simulation accuracy.
    - Implement timing simulations for sensor readings to better mimic real-world acquisition delays, jitter, and noise profiles.
- [ ] **Hardware Redundancy & Reliability**:
    - **Dual IMU Support**: Migrate MPU9250 to **SPI** (10MHz) and add a secondary redundant IMU with a voting/consensus scheme.
    - **Dual Barometers**: Implement asynchronous staggered triggering (e.g., 200Hz effective resolution) using two BMP280s on separate I2C buses.
    - **Non-blocking Sensor Acquisition**: Move from blocking I2C polling to a state-machine based "Trigger -> Process -> Collect" model to reclaim 2-4ms of CPU time per loop.
- [ ] **Advanced Apogee Control**:
    - Evolve the airbrake deployment algorithm from classical PID to **Model Predictive Control (MPC)** or **LQR**, leveraging the existing `DragTable` and `AltitudeSpeedTable` for optimal trajectory tracking.
- [ ] **Servo Trajectory Generation**:
    - Replace instant position commands with smoothed, jerk-limited trajectories to reduce power spikes, prevent stripped gears, and improve aerodynamic stability during deployment.
- [ ] **Elite Logging Optimizations**:
    - **SD Card Pre-allocation**: Implement contiguous file pre-allocation to eliminate FAT32 fragmentation stalls during high-speed writes.
    - **Full Binary SD Logging**: Move from CSV to 100% binary logging on SD to eliminate `printf` overhead and fixed-width formatting.

## Safety & Robustness (FMEA)

- [ ] **I2C Bus Recovery**: Implement a watchdog-monitored I2C recovery routine to handle bus "hangs" caused by sensor noise or electrical transients.
- [ ] **Comprehensive Abort Logic**: Audit the flight state machine for edge-case safety. Implement "Emergency Retract" or "Safe Mode" if `maxTiltAngle` is exceeded during motor burn or if estimator variance spikes.
- [ ] **Stack & Heap Monitoring**: Implement real-time monitoring of FreeRTOS task stacks and total heap usage to prevent silent overflows.

## Performance & Math Optimizations

- [ ] **Eigen Matrix Auditing**: Verify that all `Eigen` operations use fixed-size matrices to eliminate hidden heap allocations in the control loop.
- [ ] **DMA-Enabled SPI/I2C**: Once migrated to SPI, leverage ESP32 DMA for background sensor reading, minimizing the blocking time in `TaskFlightControl`.

## Sensor Calibration & Health

- [ ] **Factory Calibration Storage**: Implement a routine to save IMU/Baro calibration offsets to NVS (Non-Volatile Storage) or EEPROM. Replace the 30s "startup calibration" with a "Load or Calibrate" logic to speed up pad-ready time.
- [ ] **6-Point Tumble Calibration**: Develop an offline utility to perform a full 6-point accelerometer tumble test and ellipsoid magnetometer fitting, storing the resulting 3x3 scaling matrix and offsets.
- [ ] **Stale Data Detection**: Implement a "Watchdog for Sensors" that marks data as invalid if the sensor's internal status register doesn't update or if the values stay perfectly constant for multiple cycles.

## HIL & Simulation Pipeline

- [ ] **Closed-Loop HIL**: Transition from "Playback HIL" (CSV reading) to "Real-Time HIL" where the ESP32 sends servo commands to the Python notebook and receives updated sensor state in real-time over Serial/UDP.
- [ ] **Noise & Bias Modeling**: Update the HIL notebook to inject Gaussian noise, random walk bias, and vibration profiles into the generated `Teste_HIL_Sensors.csv` to test the Kalman Filter's robustness.
- [ ] **Automated Regression Testing**: Create a script to run multiple HIL scenarios (e.g., "High Wind," "Sensor Failure at 1km") and automatically verify if the software correctly detects apogee and deploys airbrakes.
