#pragma once
#include <cstdint>

//-----------------------------------------------------------------------------
// DTI F-SIC-A Inverter CAN Protocol
// CAN2 Runtime Integration
// 29-bit extended IDs, big-endian, 4 or 8 bytes data
// Extended ID = (Packet_ID << 8) | Node_ID
//-----------------------------------------------------------------------------

namespace dti {

//-----------------------------------------------------------------------------
// CAN Configuration
//-----------------------------------------------------------------------------
constexpr uint32_t CAN_BAUDRATE = 500'000;  // 500 kbps for CAN2
constexpr uint8_t BROADCAST_NODE_ID = 0xFF;  // Broadcast to all modules

// Build extended CAN ID from packet ID and node ID
constexpr uint32_t make_ext_id(uint8_t packet_id, uint8_t node_id) {
    return (static_cast<uint32_t>(packet_id) << 8) | node_id;
}

// Extract packet ID from extended CAN ID
constexpr uint8_t get_packet_id(uint32_t ext_id) {
    return (ext_id >> 8) & 0xFF;
}

// Extract node ID from extended CAN ID
constexpr uint8_t get_node_id(uint32_t ext_id) {
    return ext_id & 0xFF;
}

//-----------------------------------------------------------------------------
// Status Frames (Inverter -> Teensy) - ~100 Hz when enabled
//-----------------------------------------------------------------------------
namespace status {
    // Packet 0x00: ERPM, Duty, Voltage
    constexpr uint8_t ERPM_DUTY_VOLTAGE = 0x00;

    // Packet 0x01: AC current, DC current
    constexpr uint8_t AC_DC_CURRENT = 0x01;

    // Packet 0x02: Temperatures + fault code
    constexpr uint8_t TEMPS_FAULT = 0x02;

    // Packet 0x03: Id, Iq (FOC internals)
    constexpr uint8_t FOC_INTERNALS = 0x03;
}  // namespace status

//-----------------------------------------------------------------------------
// Command Frames (Teensy -> Inverter)
//-----------------------------------------------------------------------------
namespace command {
    // Primary commands
    constexpr uint8_t SET_DUTY = 0x00;              // duty * 100000 -> int32 (-1.0 to 1.0) - avoid in FS
    constexpr uint8_t SET_CURRENT = 0x01;           // target_A * 1000 -> int32 (PRIMARY for torque)
    constexpr uint8_t SET_BRAKE_CURRENT = 0x02;     // brake_A * 1000 -> int32 (positive)
    constexpr uint8_t SET_ERPM = 0x03;              // target_erpm -> int32 (speed control via PID)
    constexpr uint8_t SET_POSITION = 0x04;          // degrees * 1000000 -> int32 (requires encoder)

    // Relative current (throttle as % of max)
    constexpr uint8_t SET_RELATIVE_CURRENT = 0x0A;  // fraction * 100000 -> int32 (-1.0 to 1.0)
    constexpr uint8_t SET_RELATIVE_BRAKE = 0x0B;    // fraction * 100000 -> int32 (0.0 to 1.0)

    // Limit commands (for derating)
    constexpr uint8_t SET_MAX_AC_CURRENT = 0x0C;    // limit_A * 1000 -> int32 (max 113 Apk)
    constexpr uint8_t SET_MAX_AC_BRAKE = 0x0D;      // limit_A * 1000 -> int32
    constexpr uint8_t SET_MAX_DC_CURRENT = 0x0E;    // limit_A * 1000 -> int32 (max 55A/module)
    constexpr uint8_t SET_MAX_DC_BRAKE = 0x0F;      // limit_A * 1000 -> int32
}  // namespace command

//-----------------------------------------------------------------------------
// Fault Codes
//-----------------------------------------------------------------------------
namespace fault {
    constexpr uint8_t NONE = 0x00;                  // Normal operation
    constexpr uint8_t OVER_VOLTAGE = 0x01;          // Regen pushing bus voltage too high
    constexpr uint8_t UNDER_VOLTAGE = 0x02;         // Pack voltage too low
    constexpr uint8_t DRV = 0x03;                   // Gate driver / transistor fault
    constexpr uint8_t ABS_OVER_CURRENT = 0x04;      // AC current exceeded absolute max
    constexpr uint8_t CONTROLLER_OVER_TEMP = 0x05;  // Junction > 85°C
    constexpr uint8_t MOTOR_OVER_TEMP = 0x06;       // Motor temp > limit
    constexpr uint8_t SENSOR_WIRE_FAULT = 0x07;     // Position sensor cable issue
    constexpr uint8_t SENSOR_GENERAL_FAULT = 0x08;  // Encoder/resolver signal error
}  // namespace fault

//-----------------------------------------------------------------------------
// Scale Factors
//-----------------------------------------------------------------------------
namespace scale {
    constexpr float CURRENT_SCALE = 1000.0f;        // Amps * 1000 for commands
    constexpr float RELATIVE_CURRENT_SCALE = 100000.0f;  // Fraction * 100000
    constexpr float DUTY_SCALE = 1000.0f;           // Divide by 1000 for -1.0 to 1.0
    constexpr float AC_CURRENT_SCALE = 10.0f;       // Divide by 10 for Amps
    constexpr float DC_CURRENT_SCALE = 10.0f;       // Divide by 10 for Amps
    constexpr float TEMP_SCALE = 10.0f;             // Divide by 10 for °C
}  // namespace scale

//-----------------------------------------------------------------------------
// Hardware Limits
//-----------------------------------------------------------------------------
namespace limits {
    constexpr float MAX_DC_CURRENT_PER_MODULE = 55.0f;  // Amps (110A for 2WD, 220A for 4WD)
    constexpr float MAX_AC_CURRENT_PEAK = 113.0f;       // Apk - short duration only
    constexpr float MAX_AC_CURRENT_CONTINUOUS = 80.0f;  // Arms - cooling dependent (56-80)
    constexpr float MAX_CONTROLLER_TEMP = 85.0f;        // °C - inverter self-limits above this
    constexpr float MAX_ERPM = 200000.0f;               // 3333 Hz electrical
    constexpr float HV_INPUT_MIN = 36.0f;               // Volts
    constexpr float HV_INPUT_MAX = 600.0f;              // Volts (absolute max 620V)
    constexpr uint32_t HV_DISCHARGE_TIME_MS = 300000;   // 5 minutes after HV off
    constexpr float LV_SUPPLY_MIN = 8.0f;               // Volts
    constexpr float LV_SUPPLY_MAX = 20.0f;              // Volts
}  // namespace limits

}  // namespace dti
