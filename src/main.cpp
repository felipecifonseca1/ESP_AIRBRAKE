/**
 * @file main.cpp
 * @brief Main flight software for ESP32 Airbrake System.
 * @details Integrates IMU, Barometer, Kalman Filter, PID Control, and Data
 * Logging.
 ** @author Felipe Fonseca
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Modules
#include "FlightController.h"
#include "Config_voo.h"
#include "BMP280_HAL.h"
#include "MPU9250_HAL.h"
#include "AttitudeEstimator.h"
#include "DataManager.h"
#include "Sinalizacao.h"
#include <ArduinoEigen.h>
#include "Config_voo.h" 
#include "SystemUtils.h"

#define EEPROM_SIZE 256 // Define EEPROM size


// Flight State & Data
RawFlightData flightData;

// --- Hardware Instantiation ---
BMP280_HAL flightBaro(I2C_ADDRESS_BARO); 
MPU9250_HAL flightIMU(I2C_ADDRESS_IMU);

// --- Dependency Injection ---
AttitudeEstimator flightAttitude(&flightIMU);
FlightController &flightController = FlightController::getInstance(&flightBaro, &flightAttitude);

// --- FreeRTOS Handles ---
QueueHandle_t flightDataQueue;
SemaphoreHandle_t serialMutex;

// --- Task Functions ---
void TaskFlightControl(void *pvParameters);
void TaskLogging(void *pvParameters);
void TaskSerialComm(void *pvParameters);

void TaskFlightControl(void *pvParameters) {
  esp_task_wdt_add(NULL); // Register this task with the Watchdog
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(Ts_ms); // 20ms

  static uint16_t ledCounter = 0;
  for (;;) {
    if (++ledCounter >= 5) {
      digitalWrite(PIN_LED_1, !digitalRead(PIN_LED_1));
      ledCounter = 0;
    }
    uint32_t start_us = micros();
    static uint32_t last_start_us = 0;
    if (last_start_us > 0) {
      flightController.getDiagnosticsMutable().loopInterval_us = start_us - last_start_us;
    }
    last_start_us = start_us;

    flightController.update(); 

    RawFlightData snapshot;
    flightController.updateLogger(snapshot);
    
    uint32_t t_queue_start = micros();
    xQueueSend(flightDataQueue, &snapshot, 0); 
    flightController.getDiagnosticsMutable().queueSend_us = micros() - t_queue_start;

    esp_task_wdt_reset();                        
    vTaskDelayUntil(&xLastWakeTime, xFrequency); 
  }
}

void TaskLogging(void *pvParameters) {
  esp_task_wdt_add(NULL); // Register this task with the Watchdog

  RawFlightData dataToLog;
  DataManager &logger = DataManager::getInstance();
  FlightController &fc = FlightController::getInstance();
  uint8_t telemetryDecimation = 0;

  for (;;) {
    // Block until data arrives in the queue
    if (xQueueReceive(flightDataQueue, &dataToLog, portMAX_DELAY) == pdPASS) {
      uint32_t taskStart = micros();
      IODiagnostics& ioDiag = logger.getIODiagnosticsMutable();
      
      if (logger.isLoggingActive()) {
        logger.logFlightData(dataToLog);
      }
      
      // Offloaded Telemetry: Run at decimated rate to avoid saturating Serial
      if (++telemetryDecimation >= TELEMETRY_LOGGING_DECIMATION) {
        if (xSemaphoreTake(serialMutex, 0) == pdPASS) {
            uint32_t telStart = micros();
            fc.printFullTelemetry(dataToLog);
            uint32_t telDuration = micros() - telStart;
            
            // Average Telemetry time (Alpha = 0.1)
            if (ioDiag.telemetryPrint_us == 0) ioDiag.telemetryPrint_us = telDuration;
            else ioDiag.telemetryPrint_us = (ioDiag.telemetryPrint_us * 9 + telDuration) / 10;

            xSemaphoreGive(serialMutex);
        }
        telemetryDecimation = 0;
      }

      ioDiag.totalTaskCycle_us = micros() - taskStart;
    }
    esp_task_wdt_reset();
  }
}

void TaskSerialComm(void *pvParameters) {
  esp_task_wdt_add(NULL); // Register this task with the Watchdog
  DataManager &logger = DataManager::getInstance();
  for (;;) {
    if (Serial.available()) {
      char cmd = Serial.read();
      esp_task_wdt_reset();

      if (cmd == 'd' || cmd == 'D') {
        DEBUG_PRINTLN_F("Command received: Dump Log");
        logger.dumpSDCurrentLog();
        logger.stopLogging();
      }

      if (cmd == 'l' || cmd == 'L') {
        DEBUG_PRINTLN_F("Command received: List Files");
        logger.listSDFiles();
      }

      if (cmd == 'c' || cmd == 'C') {
        DEBUG_PRINTLN_F("Command received: Clear All Logs (SD)");
        logger.clearSDAllLogs();
      }
      // TODO: Fix current log size print while file is open
      if (cmd == 'f' || cmd == 'F') {
        DEBUG_PRINTLN_F("Command received: List Internal Flash Files");
        logger.listInternalFiles();
      }

      if (cmd == 'x' || cmd == 'X') {
        DEBUG_PRINTLN_F("Command received: CLEAR INTERNAL FLASH (Format)");
        logger.clearInternalFlash();
      }

      if (cmd == 'i' || cmd == 'I') {
        DEBUG_PRINTLN_F("Command received: Dump latest internal log...");
        logger.dumpInternalFlash(); // Defaults to -1 (latest)
      }

      if (cmd == 'p' || cmd == 'P') {
        DEBUG_PRINTLN_F("Command received: Stop Logging");
        logger.stopLogging();
      }

      if (cmd == 'w' || cmd == 'W') {
        DEBUG_PRINTLN_F("Command received: Software Reset (Simulated Crash in 2s)...");
        delay(2000); 
        esp_restart(); 
      }

      if (cmd == 'b' || cmd == 'B') {
        DEBUG_PRINTLN_F("Command received: Force Burnout State");
        flightController.forceState(FlightState::BURNOUT);
      }

      if (cmd == 't' || cmd == 'T') {
        bool newState = !flightController.isTelemetryEnabled();
        flightController.setTelemetryEnabled(newState);
        DEBUG_PRINT_F("Telemetry: ");
        DEBUG_PRINTLN(newState ? "ENABLED" : "DISABLED");
      }

      if (cmd == 'y' || cmd == 'Y') {
        DEBUG_PRINTLN_F("Command received: Internal Flash Logging ENABLED");
        logger.setInternalLogging(true);
      }

      if (cmd == 'n' || cmd == 'N') {
        DEBUG_PRINTLN_F("Command received: Internal Flash Logging DISABLED");
        logger.setInternalLogging(false);
      }

      if (cmd == 'r' || cmd == 'R') {
        DEBUG_PRINTLN_F("Command received: Reset Performance Metrics");
        flightController.resetDiagnostics();
        logger.resetIODiagnostics();
      }

      if (cmd == 'e' || cmd == 'E') {
        DEBUG_PRINTLN_F("Command received: RESET ATTITUDE ESTIMATOR");
        flightController.resetAttitudeEstimator();
      }

      if (cmd == 'm' || cmd == 'M') {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdPASS) {
            const LoopDiagnostics& diag = flightController.getDiagnostics();
            const IODiagnostics& ioDiag = logger.getIODiagnostics();
            
            float successRate = (diag.totalCycles > 0) ? 
                (1.0f - (float)diag.cyclesExceeded / diag.totalCycles) * 100.0f : 100.0f;

            DEBUG_PRINTLN_F("\n--- CORE 1: FLIGHT CONTROL ---");
            DEBUG_PRINTF("Sensor Read:      %u us\n", diag.sensorRead_us);
            DEBUG_PRINTF("Inertial (Predict):%u us\n", diag.imuFilter_us);
            DEBUG_PRINTF("Fusion (Update):   %u us\n", diag.kalmanUpdate_us);
            DEBUG_PRINTF("State Sync:        %u us\n", diag.navCalc_us);
            DEBUG_PRINTF("Queue Send:       %u us\n", diag.queueSend_us);
            DEBUG_PRINTF("Total Execution:  %u us\n", diag.totalExecute_us);
            DEBUG_PRINTF("Peak Execution:   %u us\n", diag.peakExecution_us);
            DEBUG_PRINTF("Total Cycles:     %u\n", (uint32_t)diag.totalCycles);
            DEBUG_PRINTF("Cycles Exceeded:  %u\n", (uint32_t)diag.cyclesExceeded);
            DEBUG_PRINTF("Success Rate:     %.2f %%\n", successRate);

            DEBUG_PRINTLN_F("\n--- CORE 0: I/O & LOGGING ---");
            DEBUG_PRINTF("SD (Avg Write):   %u us\n", ioDiag.sdWriteAvg_us);
            DEBUG_PRINTF("SD (Peak Hist):   %u us\n", ioDiag.maxSdWrite_us);
            DEBUG_PRINTF("Flash (Avg):      %u us\n", ioDiag.internalFlashWriteAvg_us);
            DEBUG_PRINTF("Flash (Peak):     %u us\n", ioDiag.maxInternalFlashWrite_us);
            
            // Phase-aware buffer reporting
            uint8_t state = diag.flightState; 
            if (state <= 2) { // PAD phase
                DEBUG_PRINTF("Flash Buffer:    %u / %u (Pad)\n", ioDiag.internalFlashBufferCount, LOG_PAD_FLUSH_SIZE);
            } else if (state <= 6) { // ASCENT phase
                DEBUG_PRINTF("Flash Buffer:    %u / %u (Buffered)\n", ioDiag.internalFlashBufferCount, LOG_BUFFER_SIZE_INT);
            } else { // DESCENT (7-8)
                DEBUG_PRINTF("Flash Flushing:  %u remaining\n", ioDiag.internalFlashBufferCount);
            }

            DEBUG_PRINTF("Telemetry (Avg):  %u us\n", ioDiag.telemetryPrint_us);
            DEBUG_PRINTF("Total Task Cycle: %u us\n", ioDiag.totalTaskCycle_us);
            
            xSemaphoreGive(serialMutex);
        } else {
            DEBUG_PRINTLN_F("WARNING: Serial Mutex Busy. Metrics skipped.");
        }
      }
    }
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(50)); // Poll at 20Hz
  }
}

void setup() {
  Serial.begin(115200);
  delay(500); // Give serial monitor time to connect
  
  SystemUtils::initCoreHardware(EEPROM_SIZE);

  // ! Do more recovery tests
  // Recovery check
  bool isCrashRecovery = false;
  if (useRecovery) {
    isCrashRecovery = SystemUtils::checkSystemRecovery();
  }
  bool recoverySuccess = false;

  // Create Global Mutexes
  serialMutex = xSemaphoreCreateMutex();

  // Create the Data Queue
  flightDataQueue = xQueueCreate(20, sizeof(RawFlightData));

  // Storage setup
  DEBUG_PRINTLN_F("Initializing SD card...");
  DataManager &logger = DataManager::getInstance();
  SystemUtils::verifyModule(logger.setupSD(), "SD Card", false);
  
  DEBUG_PRINTLN_F("Initializing Internal Flash...");
  SystemUtils::verifyModule(logger.setupInternalFlash(true), "LittleFS", false);

  // IMU Setup
  if (isCrashRecovery) {
      DEBUG_PRINTLN_F("BOOT: Skipping IMU Calibration for Recovery.");
      SystemUtils::verifyModule(flightIMU.init(true, false), "IMU"); 
  } else {
      SystemUtils::verifyModule(flightIMU.init(true, true), "IMU"); 
  }

  // Sensors or HIL setup
  if (HIL_MODE_ACTIVE) {
    DEBUG_PRINTLN_F("**** HIL MODE ACTIVATED ****");
    if (logger.initHIL(HIL_FILENAME)) {

      HILSimulationData firstSample = logger.readHILStep();

      if (firstSample.valid) {
        float pressaoInicial = firstSample.barometricPressure_Pa;
        flightBaro.setGroundPressureP0(pressaoInicial);

        DEBUG_PRINT_F("HIL: P0 set to: ");
        DEBUG_PRINT(pressaoInicial);
        DEBUG_PRINTLN_F(" Pa");

        logger.resetHIL();

      } else {
        DEBUG_PRINTLN_F("HIL: Error - Empty file!");
        while (1)
          ;
      }

      signalSuccessfullModule("HIL Init");
    } else {
        DEBUG_PRINTLN_F("HIL FATAL ERROR: File not found!");
        while (1); 
    }

  } else {
    DEBUG_PRINTLN_F("Initializing BMP...");
    SystemUtils::verifyModule(flightBaro.init(), "BMP280"); 
    flightBaro.calibrateGroundReference(100); // Reads n amount of times for P0
    
    // If Recovering, overwrite P0 from RTC
    if (isCrashRecovery) {
       if (flightController.attemptRecovery()) {
           recoverySuccess = true;
           DEBUG_PRINTLN_F("BOOT: **** FLIGHT RESUMED FROM RTC ****");
           signalSuccessfullModule("RESUMED");
           // Skip servo retract
       } else {
           DEBUG_PRINTLN_F("BOOT: Recovery failed (Invalid Magic). Normal startup.");
           isCrashRecovery = false; // Treat as normal
       }
    } else {
        flightController.resetRecoveryData(); // Clear old trash
    }
    
    if (!recoverySuccess) {
       DEBUG_PRINTLN_F("**** REAL FLIGHT MODE ACTIVATED ****");
       // Servo Setup
       SystemUtils::verifyModule(flightController.setupServo(), "Servo");
       flightController.retractAirbrakes();
       delay(500); 
    } else {
       flightController.setupServo(); 
    }
  }

  DEBUG_PRINTLN_F("Initializing Controller...");
  flightController.setupController();
  signalSuccessfullModule("Controller");
  delay(100);

  // Setup Kalman and Controller
  DEBUG_PRINTLN_F("Initializing Kalman Filter...");
  flightController.setupKalman();
  flightController.getAttitudeEstimator()->setUseMagnetometer(USE_MAGNETOMETER);
  flightController.getAttitudeEstimator()->setMagnetometerWeight(MAGNETOMETER_FUSION_WEIGHT);

  // Serial.flush() removed to prevent blocking on boot if monitor is closed

  if (HIL_MODE_ACTIVE) {
    DEBUG_PRINTLN("HIL Mode: Forcing state to WAIT_LAUNCH");
    flightController.forceState(FlightState::WAIT_LAUNCH);
    logger.startLogging(); // Start logging right away for HIL
  }

  SystemUtils::completeInitialization(WDT_TIMEOUT_MS);

  // Launch High-Priority Flight Task on Core 1
  xTaskCreatePinnedToCore(TaskFlightControl, "Flight", 16384, NULL, 5, NULL, 1);

  // Launch I/O Tasks on Core 0
  xTaskCreatePinnedToCore(TaskLogging, "Logging", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskSerialComm, "Serial", 8192, NULL, 3, NULL, 0);

  DEBUG_PRINT_F("System Ready. Initial State: ");
  DEBUG_PRINTLN((int)flightController.getFlightState());
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
