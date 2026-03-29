#pragma once
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <functional>

#include "data_struct.hpp"
#include "dti_can_ids.hpp"

class DTICanHandler {
public:
    DTICanHandler(SystemData& system_data,
                  volatile SystemVolatileData& volatile_updatable_data,
                  SystemVolatileData& volatile_updated_data,
                  uint8_t node_id = config::inverter::DEFAULT_NODE_ID);

    // Setup and initialization
    void setup();
    bool init();
    void reset_init();

    // Main loop functions
    void update();

    // Command functions
    void send_current(float amps);
    void send_brake_current(float amps);
    void send_erpm(int32_t erpm);
    void send_relative_current(float fraction);  // -1.0 to 1.0
    void send_relative_brake(float fraction);    // 0.0 to 1.0
    void send_duty(float duty);                  // -1.0 to 1.0 (avoid in FS)
    void stop();

    // Limit functions (for derating)
    void set_max_ac_current(float limit_amps);
    void set_max_ac_brake(float limit_amps);
    void set_max_dc_current(float limit_amps);
    void set_max_dc_brake(float limit_amps);

    // Status
    bool is_online() const { return updated_data.inverter.online; }
    uint8_t get_fault_code() const { return updated_data.inverter.temp_fault_data.fault_code; }
    const char* get_fault_name(uint8_t fault_code) const;

private:
    // CAN callback handling
    static inline std::function<void(const CAN_message_t&)> static_callback;
    static void can_rx_callback(const CAN_message_t& msg);
    void handle_can_message(const CAN_message_t& msg);

    // Status packet parsers
    void parse_erpm_duty_voltage(const uint8_t* buf, uint8_t len);
    void parse_ac_dc_current(const uint8_t* buf, uint8_t len);
    void parse_temps_fault(const uint8_t* buf, uint8_t len);
    void parse_foc_internals(const uint8_t* buf, uint8_t len);

    // Command helpers
    void send_command(uint8_t packet_id, int32_t value);
    void send_keepalive();

    // Data references
    SystemData& data;
    volatile SystemVolatileData& updatable_data;
    SystemVolatileData& updated_data;

    // CAN bus (CAN2 on Teensy 4.0/4.1)
    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

    // Configuration
    uint8_t node_id;

    // Timers
    elapsedMillis keepalive_timer;
    elapsedMillis timeout_timer;

    // State
    bool initialized = false;
};
