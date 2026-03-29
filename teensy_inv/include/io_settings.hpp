#pragma once
#include <Arduino.h>

//-----------------------------------------------------------------------------
// Debug Macros
//-----------------------------------------------------------------------------
#ifdef DEBUG_PRINTS
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(fmt, ...)
#endif

//-----------------------------------------------------------------------------
// Pin Definitions
//-----------------------------------------------------------------------------
namespace pins {

namespace can {
    // CAN2 is used for DTI inverter communication
    // CAN1 is reserved for DTI firmware updates (ignore at runtime)
    constexpr uint8_t CAN2_TX = 1;  // Teensy 4.0 CAN2 TX
    constexpr uint8_t CAN2_RX = 0;  // Teensy 4.0 CAN2 RX
}  // namespace can

namespace digital {
    // Add digital pins as needed for your setup
}  // namespace digital

namespace analog {
    // Add analog pins as needed for your setup
}  // namespace analog

}  // namespace pins

//-----------------------------------------------------------------------------
// Timing Configuration
//-----------------------------------------------------------------------------
namespace config {

namespace timing {
    constexpr uint8_t MAIN_LOOP_INTERVAL_MS = 10;      // 100 Hz main loop
    constexpr uint8_t KEEPALIVE_INTERVAL_MS = 50;      // 20 Hz minimum keepalive
    constexpr uint16_t CAN_TIMEOUT_MS = 100;           // CAN message timeout
    constexpr uint16_t INIT_TIMEOUT_MS = 2000;         // Initialization timeout
}  // namespace timing

namespace inverter {
    // Default node ID - set per module in DTI CAN Tool
    constexpr uint8_t DEFAULT_NODE_ID = 0x01;

    // Motor pole pairs - needed to convert ERPM to real RPM
    // ERPM = RPM * pole_pairs
    constexpr uint8_t MOTOR_POLE_PAIRS = 10;  // Adjust for your motor
}  // namespace inverter

}  // namespace config
