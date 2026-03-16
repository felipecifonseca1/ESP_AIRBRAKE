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
     * @brief Initializes core hardware: Serial wait, EEPROM, I2C, and Signaling.
     * @param eepromSize Size of EEPROM to allocate.
     * @return True if core initialization succeeded (currently focused on EEPROM).
     */
    bool initCoreHardware(uint16_t eepromSize);

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

} // namespace SystemUtils

#endif // SYSTEM_UTILS_H
