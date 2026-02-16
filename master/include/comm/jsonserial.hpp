#pragma once

#include <ArduinoJson.h>
#include "model/systemData.hpp"
#include "model/systemDiagnostics.hpp"
#include "logic/stateLogic.hpp"

/**
 * @brief Snapshot structure for JSON serialization to Electron dashboard
 * Maps Master's SystemData to the format expected by the UI
 */
struct DashboardSnapshot {
    uint64_t t_us = 0;  // timestamp in microseconds

    // === Digital sensors ===
    bool ebs1 = false;                  // Maps from: pneumatic_line_pressure_1_
    bool ebs2 = false;                  // Maps from: pneumatic_line_pressure_2_
    bool as_ats = false;                // Maps from: asats_pressed_
    bool ats_btn = false;               // Maps from: ats_pressed_
    bool sdc_in = false;                // Maps from: tsms_sdc_closed_
    bool asms = false;                  // Maps from: asms_on_

    // === Analog sensors ===
    float lv_soc_volts = 0.0f;          // Maps from: soc_ (needs conversion from %)
    uint16_t rear_pressure = 0;         // Maps from: _hydraulic_line_pressure
    uint16_t front_pressure = 0;        // Maps from: hydraulic_line_front_pressure

    // TODO: These sensors exist in FS-Task but need to be added to Master's HardwareData
    // uint32_t apps_higher = 0;        // APPS higher - NOT IN MASTER
    // uint32_t apps_lower = 0;         // APPS lower - NOT IN MASTER
    // uint32_t rpm_fl = 0;             // RPM front left - Master has _left_wheel_rpm (rear)
    // uint32_t rpm_fr = 0;             // RPM front right - Master has _right_wheel_rpm (rear)

    // Using Master's rear wheel RPM instead
    double rpm_rl = 0.0;                // Maps from: _left_wheel_rpm (rear left)
    double rpm_rr = 0.0;                // Maps from: _right_wheel_rpm (rear right)

    // === Additional Master sensors not in FS-Task ===
    // bool pneumatic_line_pressure_ = true;   // Combined pneumatic pressure
    // bool master_sdc_closed_ = false;        // Master SDC state
    // bool wd_ready_ = false;                 // Watchdog ready

    // === Drive mode ===
    uint8_t mode = 0;                   // Maps from: mission_

    // === Actuators (outputs) ===
    // TODO: Add actuator states from OutputCoordinator if needed
    bool brake_light = false;
    bool assi_yellow = false;
    bool assi_blue = false;
    bool ebs_solenoid1 = false;
    bool ebs_solenoid2 = false;
    bool sdc_relay = false;
};

/**
 * @brief FSM state snapshot for JSON serialization
 */
struct FsmSnapshot {
    uint8_t fsm_state = 0;              // Current state enum value
    uint32_t time_in_state_ms = 0;      // Time spent in current state
    uint8_t checkup_state = 0;          // Checkup manager state
    uint8_t ebs_state = 0;              // EBS/pressure test phase
    // TODO: Add error_messages and fault_messages vectors if needed
};

namespace jsonserial {

/**
 * @brief Populates a DashboardSnapshot from Master's SystemData
 * @param system_data Pointer to the system data
 * @param snapshot Output snapshot to populate
 */
void populateSnapshot(const SystemData* system_data, DashboardSnapshot& snapshot);

/**
 * @brief Populates an FsmSnapshot from ASState
 * @param as_state Pointer to the AS state machine
 * @param fsm_snap Output FSM snapshot to populate
 * @param checkup_state Current checkup state
 * @param ebs_state Current EBS state
 */
void populateFsmSnapshot(uint8_t current_state, uint8_t checkup_state, uint8_t ebs_state,
                         uint32_t time_in_state, FsmSnapshot& fsm_snap);

/**
 * @brief Serializes and sends snapshot data over Serial as JSON
 * @param snapshot The sensor data snapshot
 * @param fsm_snap The FSM state snapshot
 * @param failure_detection Pointer to failure detection data for error messages
 */
void send(const DashboardSnapshot& snapshot, const FsmSnapshot& fsm_snap,
          const FailureDetection* failure_detection = nullptr);

/**
 * @brief Convenience function to build and send all data in one call
 * @param system_data Pointer to system data
 * @param current_state Current master state
 * @param checkup_state Current checkup state
 * @param ebs_state Current EBS state
 */
void sendSystemData(const SystemData* system_data, uint8_t current_state,
                    uint8_t checkup_state, uint8_t ebs_state);

}  // namespace jsonserial
