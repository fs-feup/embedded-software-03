#include "comm/jsonserial.hpp"
#include <Arduino.h>

namespace jsonserial {

void populateSnapshot(const SystemData* system_data, DashboardSnapshot& snapshot) {
    snapshot.t_us = micros();

    const HardwareData& hw = system_data->hardware_data_;

    // === Digital sensors mapping ===
    snapshot.ebs1 = hw.pneumatic_line_pressure_1_;
    snapshot.ebs2 = hw.pneumatic_line_pressure_2_;
    snapshot.as_ats = hw.asats_pressed_;
    snapshot.ats_btn = hw.ats_pressed_;
    snapshot.sdc_in = hw.tsms_sdc_closed_;
    snapshot.asms = hw.asms_on_;

    // === Analog sensors mapping ===
    // SOC: Master stores as uint8_t percentage (0-100), convert to approximate voltage
    // Assuming 24V nominal battery: 23V (empty) to 27V (full)
    // voltage = 23.0 + (soc / 100.0) * 4.0
    snapshot.lv_soc_volts = 23.0f + (static_cast<float>(hw.soc_) / 100.0f) * 4.0f;

    snapshot.rear_pressure = static_cast<uint16_t>(hw._hydraulic_line_pressure);
    snapshot.front_pressure = static_cast<uint16_t>(hw.hydraulic_line_front_pressure);

    // RPM: Using Master's rear wheel RPM (it doesn't have front wheel RPM via GPIO)
    snapshot.rpm_rl = hw._left_wheel_rpm;
    snapshot.rpm_rr = hw._right_wheel_rpm;

    // === Drive mode - use Master's Mission enum directly ===
    // MANUAL=0, ACCELERATION=1, SKIDPAD=2, AUTOCROSS=3, TRACKDRIVE=4, EBS_TEST=5, INSPECTION=6, CHUCK=7
    snapshot.mode = static_cast<uint8_t>(system_data->mission_);

    // === Actuators ===
    // TODO: Get actual actuator states from DigitalSender or OutputCoordinator if needed
    // For now, derive some from hardware state
    snapshot.brake_light = false;  // TODO: Track actual state
    snapshot.assi_yellow = false;  // TODO: Track actual state
    snapshot.assi_blue = false;    // TODO: Track actual state
    snapshot.ebs_solenoid1 = hw.pneumatic_line_pressure_1_;  // Approximate from sensor
    snapshot.ebs_solenoid2 = hw.pneumatic_line_pressure_2_;  // Approximate from sensor
    snapshot.sdc_relay = hw.master_sdc_closed_;
}

void populateFsmSnapshot(uint8_t current_state, uint8_t checkup_state, uint8_t ebs_state,
                         uint32_t time_in_state, FsmSnapshot& fsm_snap) {
    // Use Master's State enum directly:
    // AS_MANUAL=0, AS_OFF=1, AS_READY=2, AS_DRIVING=3, AS_FINISHED=4, AS_EMERGENCY=5
    fsm_snap.fsm_state = current_state;
    fsm_snap.time_in_state_ms = time_in_state;
    fsm_snap.checkup_state = checkup_state;
    fsm_snap.ebs_state = ebs_state;
}

void send(const DashboardSnapshot& snapshot, const FsmSnapshot& fsm_snap,
          const FailureDetection* failure_detection) {
    JsonDocument doc;

    // Timestamp and mode
    doc["timestamp_us"] = snapshot.t_us;
    doc["mode"] = snapshot.mode;

    // --- Digital sensors ---
    JsonObject digital = doc["digital"].to<JsonObject>();
    digital["EBS1"] = snapshot.ebs1;
    digital["EBS2"] = snapshot.ebs2;
    digital["AS ATS"] = snapshot.as_ats;
    digital["ATS Btn"] = snapshot.ats_btn;
    digital["SDC IN"] = snapshot.sdc_in;
    digital["ASMS"] = snapshot.asms;

    // --- Analog sensors ---
    JsonObject analog = doc["analog"].to<JsonObject>();
    analog["LV SoC"] = snapshot.lv_soc_volts;
    analog["Rear Pressure"] = snapshot.rear_pressure;
    analog["Front Pressure"] = snapshot.front_pressure;
    analog["RPM Rear Left"] = snapshot.rpm_rl;
    analog["RPM Rear Right"] = snapshot.rpm_rr;

    // --- Validity flags ---
    JsonObject valid = doc["valid"].to<JsonObject>();
    valid["EBS1"] = true;
    valid["EBS2"] = true;
    valid["AS ATS"] = true;
    valid["ATS Btn"] = true;
    valid["SDC IN"] = true;
    valid["ASMS"] = true;
    valid["LV SoC"] = true;
    valid["Rear Pressure"] = true;
    valid["Front Pressure"] = true;
    valid["RPM Rear Left"] = true;
    valid["RPM Rear Right"] = true;
    valid["mode"] = true;

    // --- Actuators ---
    JsonObject actuators = doc["actuators"].to<JsonObject>();
    actuators["Brake Light"] = snapshot.brake_light;
    actuators["ASSI Yellow"] = snapshot.assi_yellow;
    actuators["ASSI Blue"] = snapshot.assi_blue;
    actuators["EBS Solenoid 1"] = snapshot.ebs_solenoid1;
    actuators["EBS Solenoid 2"] = snapshot.ebs_solenoid2;
    actuators["SDC Relay"] = snapshot.sdc_relay;

    // --- FSM snapshot ---
    JsonObject fsm = doc["fsm"].to<JsonObject>();
    fsm["fsm_state"] = fsm_snap.fsm_state;
    fsm["time_in_state_ms"] = fsm_snap.time_in_state_ms;
    fsm["checkup_state"] = fsm_snap.checkup_state;
    fsm["ebs_state"] = fsm_snap.ebs_state;

    // --- Error and fault messages ---
    JsonArray errors = fsm["error_messages"].to<JsonArray>();
    JsonArray faults = fsm["fault_messages"].to<JsonArray>();

    if (failure_detection != nullptr) {
        // Add errors for dead components (active failures)
        if (failure_detection->emergency_signal_) {
            errors.add("Emergency signal active");
        }
        if (failure_detection->steer_dead_) {
            faults.add("Steering timeout");
        }
        if (failure_detection->pc_dead_) {
            faults.add("PC timeout");
        }
        if (failure_detection->inversor_dead_) {
            faults.add("Inverter timeout");
        }
        if (failure_detection->res_dead_) {
            faults.add("RES signal loss");
        }
        if (failure_detection->bms_dead_) {
            faults.add("BMS timeout");
        }
    }

    // Battery state: 0=OK, 1=Warning, 2=Critical, 3=Failure
    uint8_t battery_st = 0;
    if (snapshot.lv_soc_volts < 24.0f) battery_st = 1;
    if (snapshot.lv_soc_volts < 23.5f) battery_st = 2;
    if (snapshot.lv_soc_volts < 23.0f) battery_st = 3;
    fsm["battery_st"] = battery_st;

    serializeJson(doc, Serial);
    Serial.println();
}

void sendSystemData(const SystemData* system_data, uint8_t current_state,
                    uint8_t checkup_state, uint8_t ebs_state) {
    DashboardSnapshot snapshot;
    FsmSnapshot fsm_snap;

    populateSnapshot(system_data, snapshot);

    // Calculate time in state
    static uint32_t last_state = 0;
    static uint32_t state_start_time = 0;

    if (current_state != last_state) {
        state_start_time = millis();
        last_state = current_state;
    }

    uint32_t time_in_state = millis() - state_start_time;
    populateFsmSnapshot(current_state, checkup_state, ebs_state, time_in_state, fsm_snap);

    send(snapshot, fsm_snap, &system_data->failure_detection_);
}

}  // namespace jsonserial
