#ifndef SYSTEM_UTILS_H
#define SYSTEM_UTILS_H

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <rom/rtc.h>
#include "Config_voo.h"
#include "Sinalizacao.h"

/**
 * @namespace SystemUtils
 * @brief Provides system-level helper functions for initialization, 
 *        watchdog management, and recovery checks.
 */
namespace SystemUtils {

    /**
     * @brief Prints the system header to Serial.
     */
    void printHeader();

    /**
     * @brief Initializes core hardware: Serial wait, I2C, and Signaling.
     *        Calibration data now uses Preferences NVS — no EEPROM required.
     * @return Always true; retained for call-site compatibility.
     */
    bool initCoreHardware();

    /**
     * @brief Finalizes the boot sequence.
     * @param wdtTimeout Watchdog timeout in milliseconds.
     */
    void completeInitialization(uint32_t wdtTimeout);

    /**
     * @brief Configures and stabilizes the Task Watchdog Timer.
     * @param timeout_ms Timeout period in milliseconds.
     */
    void setupWatchdog(uint32_t timeout_ms);

    /**
     * @brief Helper to verify module initialization and handle errors.
     * @param success Condition to check.
     * @param moduleName Name for logging.
     * @param fatal If true, halts system on failure.
     */
    void verifyModule(bool success, const char* moduleName, bool fatal = true);

    /**
     * @brief Checks the reset reason to decide if the system is recovering from a crash.
     * @return True if a watchdog or brownout reset was detected.
     */
    bool checkSystemRecovery();
    
    /**
     * @brief Scans the I2C bus and prints found devices to Serial.
     */
    void scanI2CBus();

    /**
     * @brief Forcefully detaches JTAG from GPIOs 39-42 to allow their use as normal GPIOs.
     * @details Crucial for ESP32-S3 if using these pins for Buzzer/Servo.
     */
    void forceDetachJTAG();

    /**
     * @brief Prints detailed hardware resource information (Flash/PSRAM) to Serial.
     */
    void checkHardwareResources();

    /**
     * @brief Performs a 9-clock-toggle I2C bus reset to free a stuck-low SDA line,
     *        then reinitializes Wire with the standard flight parameters.
     * @param sda I2C SDA pin number.
     * @param scl I2C SCL pin number.
     */
    void resetI2CBus(uint8_t sda, uint8_t scl);

} // namespace SystemUtils

#endif // SYSTEM_UTILS_H
