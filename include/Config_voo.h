#ifndef CONFIG_VOO_H
#define CONFIG_VOO_H


// CONTROLE GLOBAL DE DEBUG SERIAL
#define DEBUG_SERIAL_ENABLED 1 // 1 - debug | 0 - voo

// Variaveis globais
extern const float G_GRAVITATIONAL_CONSTANT; // Aceleração gravidade m/s^2

// MACROS DE DEBUG SERIAL

#if DEBUG_SERIAL_ENABLED == 1
    // Se o debug estiver ATIVADO:
    #include <Arduino.h> // Necessário para Serial e F()

    #define DEBUG_PRINT(...)         Serial.print(__VA_ARGS__)
    #define DEBUG_PRINTLN(...)       Serial.println(__VA_ARGS__)

    #define DEBUG_PRINT_F(str)     Serial.print(F(str))
    #define DEBUG_PRINTLN_F(str)   Serial.println(F(str))
    
    // Para placas como Teensy que suportam Serial.printf (muito útil para formatar)
    // Adicione outras definições de placas se necessário (ex: ESP32 usa Serial.printf também)
    #if defined(__IMXRT1062__) || defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY40) || defined(ESP32)
        #define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
    #else
        #define DEBUG_PRINTF(fmt, ...) \
            do { \
                Serial.print(F("[printf no dispo] ")); \
                Serial.print(F(fmt)); \
            } while(0)
        #define DEBUG_PRINTF(fmt, ...) // Alternativa: compila para nada
    #endif

#else
    // Se o debug estiver DESATIVADO:
    #define DEBUG_PRINT(...)
    #define DEBUG_PRINTLN(...)
    #define DEBUG_PRINT_F(str)
    #define DEBUG_PRINTLN_F(str)
    #define DEBUG_PRINTF(fmt, ...)
#endif // DEBUG_SERIAL_ENABLED

#endif // CONFIG_VOO_H
