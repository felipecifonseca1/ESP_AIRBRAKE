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
#include "BMP280_HAL.h"
#include "MPU9250_HAL.h"
#include "AttitudeEstimator.h"
#include "DataManager.h"
#include "Sinalizacao.h"
#include <ArduinoEigenDense.h>
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

// --- Performance Metrics ---
uint32_t loopExecTime_us = 0;
uint32_t loopUpdate_us = 0;
uint32_t loopQueue_us = 0;
uint32_t loopInterval_us = 0;
uint32_t loopPeakExec_us = 0;
uint32_t totalLoopCount = 0;

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
    if (last_start_us > 0) loopInterval_us = start_us - last_start_us;
    last_start_us = start_us;

    uint32_t t_update_start = micros();
    flightController.update(); 
    loopUpdate_us = micros() - t_update_start;

    RawFlightData snapshot;
    flightController.updateLogger(snapshot);
    
    uint32_t t_queue_start = micros();
    xQueueSend(flightDataQueue, &snapshot, 0); 
    loopQueue_us = micros() - t_queue_start;

    loopExecTime_us = micros() - start_us;
    if (loopExecTime_us > loopPeakExec_us) loopPeakExec_us = loopExecTime_us;
    totalLoopCount++;

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
      if (logger.isLoggingActive()) {
        logger.logFlightData(dataToLog);
      }
      
      // Offloaded Telemetry: Run at decimated rate to avoid saturating Serial
      if (++telemetryDecimation >= TELEMETRY_LOGGING_DECIMATION) {
        if (xSemaphoreTake(serialMutex, 0) == pdPASS) {
            fc.printFullTelemetry(dataToLog);
            xSemaphoreGive(serialMutex);
        }
        telemetryDecimation = 0;
      }
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

      if (cmd == 'm' || cmd == 'M') {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdPASS) {
            DEBUG_PRINTLN_F("\n--- PERFORMANCE METRICS ---");
            DEBUG_PRINT_F("Total Loop time:  "); DEBUG_PRINT(loopExecTime_us); DEBUG_PRINTLN_F(" us");
            DEBUG_PRINT_F("Update (Sensors): "); DEBUG_PRINT(loopUpdate_us); DEBUG_PRINTLN_F(" us");
            DEBUG_PRINT_F("Queue Send:       "); DEBUG_PRINT(loopQueue_us); DEBUG_PRINTLN_F(" us");
            DEBUG_PRINT_F("Peak execution:   "); DEBUG_PRINT(loopPeakExec_us); DEBUG_PRINTLN_F(" us");
            DEBUG_PRINT_F("Loop Interval:    "); DEBUG_PRINT(loopInterval_us); DEBUG_PRINTLN_F(" us");
            xSemaphoreGive(serialMutex);
        }
        loopPeakExec_us = 0; // Reset peak after printing
      }
    }
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(50)); // Poll at 20Hz
  }
}

void setup() {
  Serial.begin(115200);
  delay(500); // Give serial monitor time to connect
  
  // Detach JTAG and initialize core hardware
  // Non-blocking Serial is set in SystemUtils::initCoreHardware
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
  SystemUtils::verifyModule(logger.setupInternalFlash(true), "FFat", false);

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

  // Serial.flush() removed to prevent blocking on boot if monitor is closed

  if (HIL_MODE_ACTIVE) {
    DEBUG_PRINTLN("HIL Mode: Forcing state to WAIT_LAUNCH");
    flightController.forceState(FlightState::WAIT_LAUNCH);
  }

  SystemUtils::completeInitialization(WDT_TIMEOUT_MS);

  // Launch High-Priority Flight Task on Core 1
  xTaskCreatePinnedToCore(TaskFlightControl, "Flight", 8192, NULL, 5, NULL, 1);

  // Launch I/O Tasks on Core 0
  xTaskCreatePinnedToCore(TaskLogging, "Logging", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskSerialComm, "Serial", 8192, NULL, 3, NULL, 0);

  DEBUG_PRINT_F("System Ready. Initial State: ");
  DEBUG_PRINTLN((int)flightController.getFlightState());
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
