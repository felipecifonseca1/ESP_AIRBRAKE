/**
 * @file main.cpp
 * @brief Main flight software for ESP32 Airbrake System.
 * @details Integrates IMU, Barometer, Kalman Filter, PID Control, and Data
 * Logging.
 ** @author Felipe Fonseca
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <cstdint>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Modules
#include "FlightController.h"
#include "Funcoes_BMP.h"
#include "Funcoes_suporte_IMU.h"
#include "KalmanFilter.hh"

#include "Config_voo.h"
#include "DataManager.h"
#include "Sinalizacao.h"
#include <ArduinoEigenDense.h>
#define EEPROM_SIZE 256 // Define EEPROM size

uint32_t WDT_TIMEOUT_MS = 15000; // Watchdog timeout in milliseconds
// Watchdog configuration
esp_task_wdt_config_t twdt_config = {.timeout_ms = WDT_TIMEOUT_MS,
                                     .idle_core_mask = (1 << 0),
                                     .trigger_panic = true};

// Flight State & Data
RawFlightData flightData;
FlightController &flightController = FlightController::getInstance();

// --- FreeRTOS Handles ---
QueueHandle_t flightDataQueue;

// --- Task Functions ---
void TaskFlightControl(void *pvParameters);
void TaskLogging(void *pvParameters);
void TaskSerialComm(void *pvParameters);

void TaskFlightControl(void *pvParameters) {
  esp_task_wdt_add(NULL); // Register this task with the Watchdog
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(Ts_ms); // 20ms

  for (;;) {

    flightController.update(); // Update state machine and estimator 

    RawFlightData snapshot;
    flightController.updateLogger(snapshot);
    xQueueSend(flightDataQueue, &snapshot, 0); // Non-blocking send

    esp_task_wdt_reset();                        // Signal health to Watchdog
    vTaskDelayUntil(&xLastWakeTime, xFrequency); // Wait for next 20ms cycle
  }
}

void TaskLogging(void *pvParameters) {
  esp_task_wdt_add(NULL); // Register this task with the Watchdog

  RawFlightData dataToLog;
  DataManager &logger = DataManager::getInstance();

  for (;;) {
    // Block until data arrives in the queue
    if (xQueueReceive(flightDataQueue, &dataToLog, portMAX_DELAY) == pdPASS) {
      if (logger.isLoggingActive()) {
        logger.logDataSD(dataToLog);
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
        Serial.println("Command received: Dump Log");
        logger.dumpCurrentLog();
        logger.stopLogging();
      }
      if (cmd == 'h' || cmd == 'H') {
        Serial.println("Command received: Load HIL File");
        logger.stopLogging();
        logger.receiveHILFile(HIL_FILENAME);
      }

      if (cmd == 'l' || cmd == 'L') {
        Serial.println("Command received: List Files");
        logger.listFiles();
      }

      if (cmd == 'c' || cmd == 'C') {
        Serial.println("Command received: Clear All Logs");
        logger.clearAllLogs();
      }

      if (cmd == 'p' || cmd == 'P') {
        Serial.println("Command received: Stop Logging");
        logger.stopLogging();
      }
    }
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(50)); // Poll at 20Hz
  }
}

/**
 * @brief Helper to verify module initialization and handle errors.
 * @param success Condition to check
 * @param moduleName Name for logging
 * @param fatal If true, halts system on failure
 */
void verifyModule(bool success, const char* moduleName, bool fatal = true) {
    if (success) {
        signalSuccessfullModule(moduleName);
    } else {
        signalFailedModule(moduleName);
        DEBUG_PRINT_F("FATAL ERROR: Module '");
        DEBUG_PRINT_F(moduleName);
        if (moduleName == "SD Card") {
            DEBUG_PRINTLN_F("' setup failed.");
            return;
        } else {
            DEBUG_PRINTLN_F("' setup failed. System halted.");
        }
        
        if (fatal) {
            while (1) {
                // Blink Fast for Error
                ledBlink(LED_BUILTIN, 2, 100, 100); 
                delay(500);
            }
        }
    }
}

void setup() {
  Serial.begin(115200);
  // Wait for Serial to connect, with timeout
  unsigned long serialStartTime = millis();
  while (!Serial && (millis() - serialStartTime < 4000));

  DEBUG_PRINTLN_F("==== INITIALIZING AIRBRAKE SYSTEM ====");


  if (!EEPROM.begin(EEPROM_SIZE)) {
    DEBUG_PRINTLN_F("CRITICAL ERROR: Failed to initialize EEPROM!");
  } else {
    DEBUG_PRINTLN_F("EEPROM initialized successfully.");
    if (ERASE_CALIBRATION_ON_STARTUP) {
      eraseCalibration(); // First time calibration
    }
  }

  // Initialize basics: I2C, SPI and signaling system
  Wire.begin();
  Wire.setClock(400000); // Set I2C to 400kHz

  setupSinalizacao();
  signalStartupStart();



  // Create the Data Queue
  flightDataQueue = xQueueCreate(20, sizeof(RawFlightData));

  // Benchmark the SD card 
  // DataManager::getInstance().runFrequencyTest(16000000, 50, 1000, false);
  // DataManager::getInstance().runStrategyBenchmark(16000000, 1000);

  // Storage setup
  DEBUG_PRINTLN_F("Initializing SD card...");
  DataManager &logger = DataManager::getInstance();
  verifyModule(logger.setupSD(), "SD Card", false);

  // IMU Setup
  verifyModule(setup_IMU(CALIBRATE_IMU_ON_STARTUP, PERFORM_FINE_TUNING, PRINT_IMU_PARAMS), "IMU");

  // Sensors or HIL setup
  if (HIL_MODE_ACTIVE) {
    DEBUG_PRINTLN_F("**** HIL MODE ACTIVATED ****");
    if (logger.initHIL(HIL_FILENAME)) {

      HILSimulationData firstSample = logger.readHILStep();

      if (firstSample.valid) {
        float pressaoInicial = firstSample.barometricPressure_Pa;
        setGroundPressureP0_BMP(pressaoInicial);

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
    DEBUG_PRINTLN_F("Initializing BMP (setup_BMP)...");
    verifyModule(setupBMP(), "BMP280");
    DEBUG_PRINTLN_F("**** REAL FLIGHT MODE ACTIVATED ****");
  }

  // Servo Setup
  verifyModule(flightController.setupServo(), "Servo");
  flightController.retractAirbrakes();

  delay(500); // Time to stabilize data

  DEBUG_PRINTLN_F("Initializing Controller...");
  flightController.setupController();
  signalSuccessfullModule("Controller");
  delay(100);

  // Setup Kalman and Controller
  DEBUG_PRINTLN_F("Initializing Kalman Filter...");
  flightController.setupKalman();

  Serial.flush();

  if (HIL_MODE_ACTIVE) {
    DEBUG_PRINTLN("HIL Mode: Forcing state to WAIT_LAUNCH");
    flightController.forceState(FlightState::WAIT_LAUNCH);
  }

  DEBUG_PRINTLN("WDT: Initializing Watchdog...");
  esp_task_wdt_reconfigure(&twdt_config);
  DEBUG_PRINTLN("WDT: Active.");

  // Launch High-Priority Flight Task on Core 1
  xTaskCreatePinnedToCore(TaskFlightControl, "Flight", 8192, NULL, 5, NULL, 1);

  // Launch I/O Tasks on Core 0
  xTaskCreatePinnedToCore(TaskLogging, "Logging", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskSerialComm, "Serial", 4096, NULL, 3, NULL, 0);

  DEBUG_PRINTLN_F("All optional setups and tests completed.");
  signalStartupComplete();

  DEBUG_PRINT_F("System Ready. Initial State: ");
  DEBUG_PRINTLN((int)flightController.getFlightState());
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
