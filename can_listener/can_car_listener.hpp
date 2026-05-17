#pragma once

#include <cstdint>
#include <string>
#include <array>
#include "CAN_IDs.h"

namespace sensors
{
    enum class DriveMode : uint8_t
    {
        Unknown = 0,
        Manual,
        Acceleration,
        Skidpad,
        Autocross,
        Trackdrive,
        EbsTest,
        Inspection
    };

    struct Config
    {
        std::string device; // ex: /dev/cu.usbserial-DN8FHMAG  or  COM3
        int baud = 115200;

        // SLCAN CAN bus speed: S4=125k (handcart), S7=800k (accumulator), S8=1M
        // >>> TO TEST WITH HANDCART: change "S7" to "S4" <<<
        std::string slcan_speed_cmd = "S4\r"; // TEMP: 125k for handcart testing (change back to S7 for accumulator)

        // Ativa timestamp A1
        bool enable_a1_timestamp = true;

        // Timeout global do snapshot (staleness)
        uint32_t signal_timeout_ms = 200;

        // =========================================================
        // ====================== CAN IDs ===========================
        // =========================================================

        // Data logger
        uint32_t DATA_LOGGER_SIGNALS_1 = 0x511;
        uint32_t DATA_LOGGER_SIGNALS_2 = 0x512;

        // Dash
        uint32_t DASH_ID = 0x132;

        // Master
        uint32_t MASTER_ID = 0x300;

        // Bamocar
        uint32_t BAMO_RESPONSE_ID = 0x181;

        // BMS thermistor (EXTENDED 29-bit ID!)
        uint32_t BMS_THERMISTOR_ID = 0x1839F380;

        // Cells boards
        uint32_t CELL_TEMPS_BASE_ID = 0x110; // 0x110 + board
        uint32_t ALL_TEMPS_ID = 0x280;       // 0x280 + board

        // =========================================================

        float lv_soc_volts_per_lsb = 0.001f;
    };

    static constexpr uint8_t NUM_BOARDS = 6;
    static constexpr uint8_t NTC_SENSOR_COUNT = 18;
    static constexpr uint8_t TEMPS_PER_CHUNK = 6;
    static constexpr uint8_t ALL_TEMPS_CHUNKS = (NTC_SENSOR_COUNT + TEMPS_PER_CHUNK - 1) / TEMPS_PER_CHUNK; // = 3

    // ==============================
    // Subsystem: DATA_LOGGER_SIGNALS_1 (0x511)
    // ==============================
    struct Signals1
    {
        bool placeholder = false;       bool placeholder_valid = false;
        bool asms_on = false;           bool asms_on_valid = false;
        bool asats_pressed = false;     bool asats_pressed_valid = false;
        bool ats_pressed = false;       bool ats_pressed_valid = false;
        bool tsms_sdc_closed = false;   bool tsms_sdc_closed_valid = false;
        bool master_sdc_closed = false; bool master_sdc_closed_valid = false;
        bool ts_on = false;             bool ts_on_valid = false;
        bool wd_ready = false;          bool wd_ready_valid = false;

        bool emergency_signal = false;  bool emergency_signal_valid = false;
        bool bms_dead = false;          bool bms_dead_valid = false;
        bool ebs_engage = false;        bool ebs_engage_valid = false;
        bool ebs_release = false;       bool ebs_release_valid = false;
        bool steer_dead = false;        bool steer_dead_valid = false;
        bool pc_dead = false;           bool pc_dead_valid = false;
        bool inversor_dead = false;     bool inversor_dead_valid = false;
        bool res_dead = false;          bool res_dead_valid = false;

        uint8_t state_checkup = 0;      bool state_checkup_valid = false;
        bool pneu_p1 = false;           bool pneu_p1_valid = false;
        bool pneu_p2 = false;           bool pneu_p2_valid = false;
        bool pneu_p = false;            bool pneu_p_valid = false;

        uint32_t dc_voltage = 0;        bool dc_voltage_valid = false;
        uint8_t mission = 0;            bool mission_valid = false;
        uint8_t state = 0;              bool state_valid = false;
    };

    // ==============================
    // Subsystem: Hydraulic pressures
    // ==============================
    struct Hydraulic
    {
        float front_bar = 0.0f;   bool front_valid = false;
        float rear_bar = 0.0f;    bool rear_valid = false;
    };

    // ==============================
    // Subsystem: DASH_ID (0x132)  buf[0] = type mux
    // ==============================
    struct Dash
    {
        uint8_t driving_state = 0;   bool driving_state_valid = false;
        bool implausibility = false; bool implausibility_valid = false;
        uint16_t brake_pressure = 0; bool brake_pressure_valid = false;
        int32_t apps_higher = 0;     bool apps_higher_valid = false;
        int32_t apps_lower = 0;      bool apps_lower_valid = false;
        uint32_t rpm_fr_raw = 0;     bool rpm_fr_valid = false;
        uint32_t rpm_fl_raw = 0;     bool rpm_fl_valid = false;
    };

    // ==============================
    // Subsystem: MASTER_ID (0x300) buf[0] = type mux
    // ==============================
    struct Master
    {
        uint16_t brake_pressure = 0;     bool brake_pressure_valid = false;
        bool asms_on = false;            bool asms_on_valid = false;
        uint8_t soc = 0;                bool soc_valid = false;
        uint8_t as_state = 0;           bool as_state_valid = false;
        uint8_t autonomous_mission = 0; bool autonomous_mission_valid = false;
    };

    // ==============================
    // Subsystem: BAMOCAR responses (0x181)
    // ==============================
    struct Bamocar
    {
        bool ts_on = false;                bool ts_on_valid = false;
        int32_t speed = 0;                 bool speed_valid = false;
        int32_t motor_current = 0;         bool motor_current_valid = false;
        uint16_t error_bitmap = 0;         bool error_bitmap_valid = false;
        uint16_t warning_bitmap = 0;       bool warning_bitmap_valid = false;
        bool btb_ready = false;            bool btb_ready_valid = false;
        bool transmission_enabled = false; bool transmission_enabled_valid = false;
    };

    // ==============================
    // Subsystem: BMS thermistor summary (extended 0x1839F380)
    // ==============================
    struct BmsThermistors
    {
        int8_t min_temp = 0;              bool min_temp_valid = false;
        int8_t max_temp = 0;              bool max_temp_valid = false;
        int8_t avg_temp = 0;              bool avg_temp_valid = false;
        uint8_t module_number = 0;        bool module_number_valid = false;
        uint8_t number_of_thermistors = 0; bool number_of_thermistors_valid = false;
    };

    // ==============================
    // Subsystem: Cells boards (0x110+board) + ALL_TEMPS (0x280+board)
    // ==============================
    struct Cells
    {
        std::array<int8_t, NUM_BOARDS> board_min{};
        std::array<int8_t, NUM_BOARDS> board_max{};
        std::array<int8_t, NUM_BOARDS> board_avg{};
        std::array<bool, NUM_BOARDS> board_valid{};

        std::array<std::array<int8_t, NTC_SENSOR_COUNT>, NUM_BOARDS> all{};
        std::array<uint8_t, NUM_BOARDS> chunks_mask{};
        std::array<bool, NUM_BOARDS> all_valid{};

        int8_t global_min = 0;  bool global_min_valid = false;
        int8_t global_max = 0;  bool global_max_valid = false;
    };

    // ==============================
    // Unified Snapshot
    // ==============================
    struct Snapshot
    {
        uint64_t t_us = 0;

        Signals1 signals1;
        Hydraulic hydraulic;
        Dash dash;
        Master master;
        Bamocar bamocar;
        BmsThermistors bms_thermistors;
        Cells cells;
    };

    bool begin(const Config &cfg);
    void tick();
    Snapshot read();
    void stop();
}
