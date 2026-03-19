#include "SystemUtils.h"
#include "Config_voo.h"
#include "Sinalizacao.h"
#include <Arduino.h>
#include <esp_task_wdt.h>
#include "driver/gpio.h"

#include <Wire.h>
#include <EEPROM.h>

namespace SystemUtils {

    void printHeader() {
        DEBUG_PRINTLN_F("==== INITIALIZING AIRBRAKE SYSTEM ====");
    }

    bool initCoreHardware(uint16_t eepromSize) {
        forceDetachJTAG(); // Force JTAG pins to GPIO mode first
        
        // UART Serial (CDC_ON_BOOT=0): immune to monitor connect/disconnect resets.
        Serial.setTimeout(0);  // Non-blocking reads
        delay(100);
        
        printHeader();
        
        bool eepromSuccess = EEPROM.begin(eepromSize);
        if (!eepromSuccess) {
            DEBUG_PRINTLN_F("CRITICAL ERROR: Failed to initialize EEPROM!");
        } else {
            DEBUG_PRINTLN_F("EEPROM initialized successfully.");
        }

        // Initialize buses
        Wire.begin(PIN_SDA, PIN_SCL);
        delay(200);
        
        if (runBusScan) {
            scanI2CBus();
        }

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
                    ledBlink(PIN_LED_1, 2, 100, 100); 
                    delay(500);
                }
            }
        }
    }

    bool checkSystemRecovery() {
        RESET_REASON reason = rtc_get_reset_reason(0);
        DEBUG_PRINT_F("BOOT: Reset Reason CPU0: "); 
        DEBUG_PRINTLN(reason);
      
        // Standard ESP32/S3 reset reasons
        if (reason == POWERON_RESET || reason == RTCWDT_RTC_RESET || reason == RTCWDT_BROWN_OUT_RESET || reason == RTCWDT_CPU_RESET) {
             DEBUG_PRINTLN_F("BOOT: WATCHDOG/BROWNOUT/SW RESET DETECTED! Attempting Recovery...");
             return true;
        }
        return false;
    }

    void scanI2CBus() {
        DEBUG_PRINTLN("Searching for i2c devices...");
        int nDevices = 0;
        for (uint8_t i = 1; i < 127; ++i) {
            Wire.beginTransmission(i);
            if (Wire.endTransmission() == 0) {
                DEBUG_PRINT("I2C device found at address 0x");
                if (i < 16) DEBUG_PRINT("0");
                DEBUG_PRINTLN_F(""); // Just for newline or HEX
                DEBUG_PRINTF("0x%02X\n", i);
                nDevices++;
            }
        }
        if (nDevices == 0) {
            DEBUG_PRINTLN("No I2C devices found\n");
        } else {
            DEBUG_PRINTLN("done\n");
        }
    }

    void forceDetachJTAG() {
        // ESP32-S3 JTAG pins: 39=MTCK, 40=MTDI, 41=MTDO, 42=MTMS
        #if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ESP32S3)

        // Reset pins to set them to a clean state first
        gpio_reset_pin((gpio_num_t)39);
        gpio_reset_pin((gpio_num_t)40);
        gpio_reset_pin((gpio_num_t)41);
        gpio_reset_pin((gpio_num_t)42);

        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << 39) | (1ULL << 40) | (1ULL << 41) | (1ULL << 42);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE; 
        gpio_config(&io_conf);
        #endif
        DEBUG_PRINTLN_F("System: JTAG pins detached and reset to GPIO mode.");
    }

} // namespace SystemUtils
