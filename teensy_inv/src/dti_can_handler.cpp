#include "dti_can_handler.hpp"
#include "io_settings.hpp"

DTICanHandler::DTICanHandler(SystemData& system_data,
                             volatile SystemVolatileData& volatile_updatable_data,
                             SystemVolatileData& volatile_updated_data,
                             uint8_t node_id)
    : data(system_data),
      updatable_data(volatile_updatable_data),
      updated_data(volatile_updated_data),
      node_id(node_id) {
    static_callback = [this](const CAN_message_t& msg) { this->handle_can_message(msg); };
}

void DTICanHandler::setup() {
    DEBUG_PRINTLN("Setting up DTI CAN handler...");

    can2.begin();
    can2.setBaudRate(dti::CAN_BAUDRATE);
    can2.setMaxMB(16);
    can2.enableFIFO();
    can2.enableFIFOInterrupt();
    can2.onReceive(can_rx_callback);

    // Accept all extended ID messages from our node
    // Filter will be set based on node_id
    can2.setFIFOFilter(ACCEPT_ALL);

    delay(100);
    DEBUG_PRINTLN("DTI CAN handler setup complete");
}

bool DTICanHandler::init() {
    // Check if we're receiving status messages from the inverter
    if (updated_data.inverter.online) {
        initialized = true;
        data.state = InverterState::READY;
        DEBUG_PRINTLN("DTI inverter initialized - receiving status messages");
        return true;
    }

    // Timeout check
    if (timeout_timer >= config::timing::INIT_TIMEOUT_MS) {
        data.state = InverterState::ERROR;
        DEBUG_PRINTLN("DTI inverter init timeout - no status messages received");
        return false;
    }

    data.state = InverterState::INITIALIZING;
    return false;
}

void DTICanHandler::reset_init() {
    initialized = false;
    timeout_timer = 0;
    data.state = InverterState::IDLE;
}

void DTICanHandler::update() {
    // Copy volatile data
    copy_volatile_data(updated_data, updatable_data);

    // Check for timeout (inverter offline)
    if (millis() - updated_data.last_rx_time > config::timing::CAN_TIMEOUT_MS) {
        if (updated_data.inverter.online) {
            DEBUG_PRINTLN("DTI inverter offline - no messages received");
        }
        updatable_data.inverter.online = false;
        updated_data.inverter.online = false;
    }

    // Send keepalive if torque is enabled
    if (data.torque_enabled && keepalive_timer >= config::timing::KEEPALIVE_INTERVAL_MS) {
        send_keepalive();
        keepalive_timer = 0;
    }

    // Handle faults
    if (updated_data.inverter.temp_fault_data.fault_code != dti::fault::NONE) {
        data.state = InverterState::FAULT;
    }
}

void DTICanHandler::can_rx_callback(const CAN_message_t& msg) {
    if (static_callback) {
        static_callback(msg);
    }
}

void DTICanHandler::handle_can_message(const CAN_message_t& msg) {
    // Only process extended ID messages
    if (!(msg.flags.extended)) {
        return;
    }

    uint8_t packet_id = dti::get_packet_id(msg.id);
    uint8_t msg_node_id = dti::get_node_id(msg.id);

    // Filter by node ID (or accept broadcast)
    if (msg_node_id != node_id && msg_node_id != dti::BROADCAST_NODE_ID) {
        return;
    }

    // Update timestamp
    updatable_data.last_rx_time = millis();
    updatable_data.inverter.online = true;
    updatable_data.inverter.last_update_ms = millis();

    // Parse based on packet ID
    switch (packet_id) {
        case dti::status::ERPM_DUTY_VOLTAGE:
            parse_erpm_duty_voltage(msg.buf, msg.len);
            break;
        case dti::status::AC_DC_CURRENT:
            parse_ac_dc_current(msg.buf, msg.len);
            break;
        case dti::status::TEMPS_FAULT:
            parse_temps_fault(msg.buf, msg.len);
            break;
        case dti::status::FOC_INTERNALS:
            parse_foc_internals(msg.buf, msg.len);
            break;
        default:
            break;
    }
}

void DTICanHandler::parse_erpm_duty_voltage(const uint8_t* buf, uint8_t len) {
    if (len < 8) return;

    // Big-endian parsing
    // Bytes 0-1: DC voltage (int16)
    // Bytes 2-3: Duty cycle (int16, /1000)
    // Bytes 4-7: ERPM (int32)

    int16_t voltage_raw = (buf[0] << 8) | buf[1];
    int16_t duty_raw = (buf[2] << 8) | buf[3];
    int32_t erpm = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7];

    updatable_data.inverter.erpm_data.dc_voltage = static_cast<float>(voltage_raw);
    updatable_data.inverter.erpm_data.duty_cycle = static_cast<float>(duty_raw) / dti::scale::DUTY_SCALE;
    updatable_data.inverter.erpm_data.erpm = erpm;

    DEBUG_PRINTF("ERPM: %ld, Duty: %.2f, Voltage: %.1fV\n",
                 erpm, updatable_data.inverter.erpm_data.duty_cycle,
                 updatable_data.inverter.erpm_data.dc_voltage);
}

void DTICanHandler::parse_ac_dc_current(const uint8_t* buf, uint8_t len) {
    if (len < 4) return;

    // Big-endian parsing
    // Bytes 0-1: AC current (int16, /10)
    // Bytes 2-3: DC current (int16, /10)

    int16_t ac_raw = (buf[0] << 8) | buf[1];
    int16_t dc_raw = (buf[2] << 8) | buf[3];

    updatable_data.inverter.current_data.ac_current =
        static_cast<float>(ac_raw) / dti::scale::AC_CURRENT_SCALE;
    updatable_data.inverter.current_data.dc_current =
        static_cast<float>(dc_raw) / dti::scale::DC_CURRENT_SCALE;

    DEBUG_PRINTF("AC: %.1fA, DC: %.1fA\n",
                 updatable_data.inverter.current_data.ac_current,
                 updatable_data.inverter.current_data.dc_current);
}

void DTICanHandler::parse_temps_fault(const uint8_t* buf, uint8_t len) {
    if (len < 5) return;

    // Byte 0: Fault code
    // Bytes 1-2: Motor temp (int16, /10)
    // Bytes 3-4: Controller temp (int16, /10)

    uint8_t fault = buf[0];
    int16_t motor_temp_raw = (buf[1] << 8) | buf[2];
    int16_t ctrl_temp_raw = (buf[3] << 8) | buf[4];

    updatable_data.inverter.temp_fault_data.fault_code = fault;
    updatable_data.inverter.temp_fault_data.motor_temp =
        static_cast<float>(motor_temp_raw) / dti::scale::TEMP_SCALE;
    updatable_data.inverter.temp_fault_data.controller_temp =
        static_cast<float>(ctrl_temp_raw) / dti::scale::TEMP_SCALE;

    if (fault != dti::fault::NONE) {
        DEBUG_PRINTF("FAULT: 0x%02X (%s)\n", fault, get_fault_name(fault));
    }

    DEBUG_PRINTF("Motor: %.1f°C, Controller: %.1f°C\n",
                 updatable_data.inverter.temp_fault_data.motor_temp,
                 updatable_data.inverter.temp_fault_data.controller_temp);
}

void DTICanHandler::parse_foc_internals(const uint8_t* buf, uint8_t len) {
    if (len < 8) return;

    // Bytes 0-3: Id (int32, /100)
    // Bytes 4-7: Iq (int32, /100)

    int32_t id_raw = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    int32_t iq_raw = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7];

    updatable_data.inverter.foc_data.id = static_cast<float>(id_raw) / 100.0f;
    updatable_data.inverter.foc_data.iq = static_cast<float>(iq_raw) / 100.0f;
}

void DTICanHandler::send_command(uint8_t packet_id, int32_t value) {
    CAN_message_t msg;
    msg.id = dti::make_ext_id(packet_id, node_id);
    msg.flags.extended = true;
    msg.len = 4;

    // Big-endian encoding
    msg.buf[0] = (value >> 24) & 0xFF;
    msg.buf[1] = (value >> 16) & 0xFF;
    msg.buf[2] = (value >> 8) & 0xFF;
    msg.buf[3] = value & 0xFF;

    can2.write(msg);
}

void DTICanHandler::send_current(float amps) {
    int32_t value = static_cast<int32_t>(amps * dti::scale::CURRENT_SCALE);
    send_command(dti::command::SET_CURRENT, value);
    DEBUG_PRINTF("Sending current: %.2fA (raw: %ld)\n", amps, value);
}

void DTICanHandler::send_brake_current(float amps) {
    // Brake current is always positive
    int32_t value = static_cast<int32_t>(fabsf(amps) * dti::scale::CURRENT_SCALE);
    send_command(dti::command::SET_BRAKE_CURRENT, value);
    DEBUG_PRINTF("Sending brake current: %.2fA\n", fabsf(amps));
}

void DTICanHandler::send_erpm(int32_t erpm) {
    send_command(dti::command::SET_ERPM, erpm);
    DEBUG_PRINTF("Sending ERPM: %ld\n", erpm);
}

void DTICanHandler::send_relative_current(float fraction) {
    // Clamp to -1.0 to 1.0
    fraction = constrain(fraction, -1.0f, 1.0f);
    int32_t value = static_cast<int32_t>(fraction * dti::scale::RELATIVE_CURRENT_SCALE);
    send_command(dti::command::SET_RELATIVE_CURRENT, value);
    DEBUG_PRINTF("Sending relative current: %.2f%%\n", fraction * 100.0f);
}

void DTICanHandler::send_relative_brake(float fraction) {
    // Clamp to 0.0 to 1.0
    fraction = constrain(fraction, 0.0f, 1.0f);
    int32_t value = static_cast<int32_t>(fraction * dti::scale::RELATIVE_CURRENT_SCALE);
    send_command(dti::command::SET_RELATIVE_BRAKE, value);
    DEBUG_PRINTF("Sending relative brake: %.2f%%\n", fraction * 100.0f);
}

void DTICanHandler::send_duty(float duty) {
    // Clamp to -1.0 to 1.0 (avoid using in FS - prefer current control)
    duty = constrain(duty, -1.0f, 1.0f);
    int32_t value = static_cast<int32_t>(duty * dti::scale::RELATIVE_CURRENT_SCALE);
    send_command(dti::command::SET_DUTY, value);
    DEBUG_PRINTF("Sending duty: %.2f%%\n", duty * 100.0f);
}

void DTICanHandler::stop() {
    send_current(0.0f);
    data.torque_enabled = false;
    DEBUG_PRINTLN("Inverter stopped");
}

void DTICanHandler::set_max_ac_current(float limit_amps) {
    int32_t value = static_cast<int32_t>(limit_amps * dti::scale::CURRENT_SCALE);
    send_command(dti::command::SET_MAX_AC_CURRENT, value);
    data.limits.max_ac_current = limit_amps;
    DEBUG_PRINTF("Set max AC current: %.1fA\n", limit_amps);
}

void DTICanHandler::set_max_ac_brake(float limit_amps) {
    int32_t value = static_cast<int32_t>(limit_amps * dti::scale::CURRENT_SCALE);
    send_command(dti::command::SET_MAX_AC_BRAKE, value);
    data.limits.max_ac_brake = limit_amps;
    DEBUG_PRINTF("Set max AC brake: %.1fA\n", limit_amps);
}

void DTICanHandler::set_max_dc_current(float limit_amps) {
    // Enforce hardware limit
    limit_amps = min(limit_amps, dti::limits::MAX_DC_CURRENT_PER_MODULE);
    int32_t value = static_cast<int32_t>(limit_amps * dti::scale::CURRENT_SCALE);
    send_command(dti::command::SET_MAX_DC_CURRENT, value);
    data.limits.max_dc_current = limit_amps;
    DEBUG_PRINTF("Set max DC current: %.1fA\n", limit_amps);
}

void DTICanHandler::set_max_dc_brake(float limit_amps) {
    int32_t value = static_cast<int32_t>(limit_amps * dti::scale::CURRENT_SCALE);
    send_command(dti::command::SET_MAX_DC_BRAKE, value);
    data.limits.max_dc_brake = limit_amps;
    DEBUG_PRINTF("Set max DC brake: %.1fA\n", limit_amps);
}

void DTICanHandler::send_keepalive() {
    // Send 0A current as keepalive to prevent timeout
    if (!data.torque_enabled || data.target_current == 0.0f) {
        send_current(0.0f);
    }
}

const char* DTICanHandler::get_fault_name(uint8_t fault_code) const {
    switch (fault_code) {
        case dti::fault::NONE:
            return "None";
        case dti::fault::OVER_VOLTAGE:
            return "Over Voltage";
        case dti::fault::UNDER_VOLTAGE:
            return "Under Voltage";
        case dti::fault::DRV:
            return "DRV (Gate Driver)";
        case dti::fault::ABS_OVER_CURRENT:
            return "Abs Over Current";
        case dti::fault::CONTROLLER_OVER_TEMP:
            return "Controller Over Temp";
        case dti::fault::MOTOR_OVER_TEMP:
            return "Motor Over Temp";
        case dti::fault::SENSOR_WIRE_FAULT:
            return "Sensor Wire Fault";
        case dti::fault::SENSOR_GENERAL_FAULT:
            return "Sensor General Fault";
        default:
            return "Unknown";
    }
}
