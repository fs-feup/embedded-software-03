#pragma once
#include <cstdint>

//-----------------------------------------------------------------------------
// DTI Inverter Data Structures
//-----------------------------------------------------------------------------

// Status data received from inverter (Packet 0x00)
struct InverterERPMData {
    int32_t erpm = 0;           // Electrical RPM (divide by pole pairs for real RPM)
    float duty_cycle = 0.0f;    // -1.0 to +1.0 (negative = regenerating)
    float dc_voltage = 0.0f;    // DC bus voltage in Volts
};

// Current data received from inverter (Packet 0x01)
struct InverterCurrentData {
    float ac_current = 0.0f;    // Motor phase current in Amps (negative = regen)
    float dc_current = 0.0f;    // Battery-side current in Amps
};

// Temperature and fault data received from inverter (Packet 0x02)
struct InverterTempFaultData {
    uint8_t fault_code = 0;     // 0x00 = fine, anything else = motor stopped
    float motor_temp = 0.0f;    // Motor temperature in °C
    float controller_temp = 0.0f; // Controller temperature in °C (max 85°C)
};

// FOC internals (Packet 0x03) - optional advanced diagnostics
struct InverterFOCData {
    float id = 0.0f;            // Should be ~0 in normal operation
    float iq = 0.0f;            // Approximately proportional to torque
};

// Complete inverter status for one module
struct InverterStatus {
    InverterERPMData erpm_data;
    InverterCurrentData current_data;
    InverterTempFaultData temp_fault_data;
    InverterFOCData foc_data;

    uint32_t last_update_ms = 0;    // Timestamp of last received message
    bool online = false;             // True if receiving status messages
};

// Command limits (for derating)
struct InverterLimits {
    float max_ac_current = 0.0f;    // Max motor current limit
    float max_ac_brake = 0.0f;      // Max regen motor current
    float max_dc_current = 55.0f;   // Max battery current (55A per module max)
    float max_dc_brake = 0.0f;      // Max regen battery current
};

// Inverter state machine states
enum class InverterState : uint8_t {
    IDLE,               // Not initialized
    INITIALIZING,       // Waiting for status messages
    READY,              // Receiving status, ready for commands
    RUNNING,            // Actively commanding torque
    FAULT,              // Fault detected
    ERROR               // Communication error
};

// Main system data structure
struct SystemData {
    InverterStatus inverter;        // Single inverter for now (expand to array for multi-motor)
    InverterLimits limits;
    InverterState state = InverterState::IDLE;

    // Command values
    float target_current = 0.0f;    // Target current in Amps
    float brake_current = 0.0f;     // Brake current in Amps
    int32_t target_erpm = 0;        // Target ERPM (for speed control mode)

    // Flags
    bool torque_enabled = false;
    bool use_speed_control = false;
};

// Volatile data updated from CAN interrupts
struct SystemVolatileData {
    InverterStatus inverter;
    uint32_t last_rx_time = 0;
};

// Copy volatile data safely
inline void copy_volatile_data(SystemVolatileData& dest, volatile SystemVolatileData& src) {
    noInterrupts();
    dest = src;
    interrupts();
}
