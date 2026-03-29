#include <Arduino.h>

#include "data_struct.hpp"
#include "dti_can_handler.hpp"
#include "io_settings.hpp"

//-----------------------------------------------------------------------------
// Global Data
//-----------------------------------------------------------------------------
SystemData data;
SystemVolatileData updated_data;
volatile SystemVolatileData updatable_data;

//-----------------------------------------------------------------------------
// Handlers
//-----------------------------------------------------------------------------
DTICanHandler can_handler(data, updatable_data, updated_data);

//-----------------------------------------------------------------------------
// Timers
//-----------------------------------------------------------------------------
elapsedMillis loop_timer;

//-----------------------------------------------------------------------------
// Setup
//-----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(100);

    DEBUG_PRINTLN("========================================");
    DEBUG_PRINTLN("DTI Inverter Controller - Teensy");
    DEBUG_PRINTLN("========================================");

    can_handler.setup();

    DEBUG_PRINTLN("Setup complete. Waiting for inverter...");
}

//-----------------------------------------------------------------------------
// Main Loop
//-----------------------------------------------------------------------------
void loop() {
    if (loop_timer >= config::timing::MAIN_LOOP_INTERVAL_MS) {
        loop_timer = 0;

        // Update CAN handler (processes incoming messages, sends keepalive)
        can_handler.update();

        // State machine
        switch (data.state) {
            case InverterState::IDLE:
                // Try to initialize
                can_handler.init();
                break;

            case InverterState::INITIALIZING:
                // Still waiting for status messages
                can_handler.init();
                break;

            case InverterState::READY:
                // Inverter is ready, can accept commands
                // Add your control logic here
                break;

            case InverterState::RUNNING:
                // Actively controlling torque
                // Add your torque control logic here
                break;

            case InverterState::FAULT:
                // Handle fault
                DEBUG_PRINTF("FAULT: %s\n", can_handler.get_fault_name(can_handler.get_fault_code()));
                can_handler.stop();
                // Could add fault recovery logic here
                break;

            case InverterState::ERROR:
                // Communication error
                DEBUG_PRINTLN("Communication error - retrying...");
                can_handler.reset_init();
                break;
        }

        // Debug output every second
        static elapsedMillis debug_timer;
        if (debug_timer >= 1000) {
            debug_timer = 0;

            DEBUG_PRINTLN("----------------------------------------");
            DEBUG_PRINTF("State: %d | Online: %s\n",
                         static_cast<int>(data.state),
                         can_handler.is_online() ? "YES" : "NO");

            if (can_handler.is_online()) {
                DEBUG_PRINTF("ERPM: %ld | RPM: %ld\n",
                             updated_data.inverter.erpm_data.erpm,
                             updated_data.inverter.erpm_data.erpm / config::inverter::MOTOR_POLE_PAIRS);
                DEBUG_PRINTF("Voltage: %.1fV | AC: %.1fA | DC: %.1fA\n",
                             updated_data.inverter.erpm_data.dc_voltage,
                             updated_data.inverter.current_data.ac_current,
                             updated_data.inverter.current_data.dc_current);
                DEBUG_PRINTF("Motor: %.1f°C | Controller: %.1f°C\n",
                             updated_data.inverter.temp_fault_data.motor_temp,
                             updated_data.inverter.temp_fault_data.controller_temp);
            }
        }
    }
}
