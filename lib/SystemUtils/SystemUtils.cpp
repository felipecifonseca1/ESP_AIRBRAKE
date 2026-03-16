#include "SystemUtils.h"
#include "Config_voo.h"
#include "Sinalizacao.h"
#include <Arduino.h>
#include <esp_task_wdt.h>

#include <Wire.h>
#include <EEPROM.h>

namespace SystemUtils {

    void printHeader() {
        DEBUG_PRINTLN_F("==== INITIALIZING AIRBRAKE SYSTEM ====");
    }

    bool initCoreHardware(uint16_t eepromSize) {
        // Wait for Serial to connect, with timeout
        unsigned long serialStartTime = millis();
        while (!Serial && (millis() - serialStartTime < 4000));
        
        printHeader();

        bool eepromSuccess = EEPROM.begin(eepromSize);
        if (!eepromSuccess) {
            DEBUG_PRINTLN_F("CRITICAL ERROR: Failed to initialize EEPROM!");
        } else {
            DEBUG_PRINTLN_F("EEPROM initialized successfully.");
        }

        // Initialize buses
        Wire.begin();
        Wire.setClock(400000); // 400kHz

        // Signaling setup
        setupSinalizacao();
        signalStartupStart();

        return eepromSuccess;
    }

    void completeInitialization(uint32_t wdtTimeout) {
        DEBUG_PRINTLN("WDT: Initializing Watchdog...");
        setupWatchdog(wdtTimeout);
        DEBUG_PRINTLN("WDT: Active.");

        DEBUG_PRINTLN_F("All optional setups and tests completed.");
        signalStartupComplete();
    }

    void setupWatchdog(uint32_t timeout_ms) {
        esp_task_wdt_config_t twdt_config = {
            .timeout_ms = timeout_ms,
            .idle_core_mask = (1 << 1), // Watch Core 1 idle task, ignore Core 0
            .trigger_panic = true
        };
        esp_task_wdt_reconfigure(&twdt_config);
    }

    void verifyModule(bool success, const char* moduleName, bool fatal) {
        if (success) {
            signalSuccessfullModule(moduleName);
        } else {
            signalFailedModule(moduleName);
            DEBUG_PRINT_F("FATAL ERROR: Module '");
            DEBUG_PRINT_F(moduleName);
            
            if (strcmp(moduleName, "SD Card") == 0) {
                DEBUG_PRINTLN_F("' setup failed.");
                return;
            } else {
                DEBUG_PRINTLN_F("' setup failed. System halted.");
            }
            
            if (fatal) {
                while (1) {
                    ledBlink(LED_BUILTIN, 2, 100, 100); 
                    delay(500);
                }
            }
        }
    }

    bool checkSystemRecovery() {
        RESET_REASON reason = rtc_get_reset_reason(0);
        DEBUG_PRINT_F("BOOT: Reset Reason CPU0: "); 
        DEBUG_PRINTLN(reason);
      
        // RTCWDT_RTC_RESET = 1, TG0WDT_SYS_RESET = 8, SW_CPU_RESET = 12, RTCWDT_BROWN_OUT_RESET = 15
        if (reason == TG0WDT_SYS_RESET || reason == RTCWDT_RTC_RESET || reason == RTCWDT_BROWN_OUT_RESET || reason == SW_CPU_RESET) {
             DEBUG_PRINTLN_F("BOOT: WATCHDOG/BROWNOUT RESET DETECTED! Attempting Recovery...");
             return true;
        }
        return false;
    }

} // namespace SystemUtils
