#include "can_car_listener.hpp"

#include <atomic>
#include <csignal>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

static std::atomic<bool> g_run{true};

static void on_sigint(int)
{
    g_run.store(false, std::memory_order_relaxed);
}

static void install_signal_handlers()
{
    std::signal(SIGINT, on_sigint);
#ifdef SIGTERM
    std::signal(SIGTERM, on_sigint);
#endif
}

static inline const char* jbool(bool v) { return v ? "true" : "false"; }

static std::ostream& json_kv(std::ostream& os, const char* k, uint64_t v, bool last = false)
{
    os << '"' << k << "\":" << v;
    if (!last) os << ',';
    return os;
}
static std::ostream& json_kv(std::ostream& os, const char* k, int64_t v, bool last = false)
{
    os << '"' << k << "\":" << v;
    if (!last) os << ',';
    return os;
}
static std::ostream& json_kv(std::ostream& os, const char* k, double v, bool last = false)
{
    os << '"' << k << "\":" << std::setprecision(10) << v;
    if (!last) os << ',';
    return os;
}
static std::ostream& json_kv(std::ostream& os, const char* k, bool v, bool last = false)
{
    os << '"' << k << "\":" << jbool(v);
    if (!last) os << ',';
    return os;
}

static void print_snapshot_json(const sensors::Snapshot& s)
{
    std::cout << '{';

    json_kv(std::cout, "t_us", s.t_us);

    // ==== Signals1 ====
    json_kv(std::cout, "asms_on", s.signals1.asms_on);
    json_kv(std::cout, "asms_on_valid", s.signals1.asms_on_valid);
    json_kv(std::cout, "asats_pressed", s.signals1.asats_pressed);
    json_kv(std::cout, "asats_pressed_valid", s.signals1.asats_pressed_valid);
    json_kv(std::cout, "ats_pressed", s.signals1.ats_pressed);
    json_kv(std::cout, "ats_pressed_valid", s.signals1.ats_pressed_valid);
    json_kv(std::cout, "tsms_sdc_closed", s.signals1.tsms_sdc_closed);
    json_kv(std::cout, "tsms_sdc_closed_valid", s.signals1.tsms_sdc_closed_valid);
    json_kv(std::cout, "master_sdc_closed", s.signals1.master_sdc_closed);
    json_kv(std::cout, "master_sdc_closed_valid", s.signals1.master_sdc_closed_valid);
    json_kv(std::cout, "ts_on", s.signals1.ts_on);
    json_kv(std::cout, "ts_on_valid", s.signals1.ts_on_valid);
    json_kv(std::cout, "wd_ready", s.signals1.wd_ready);
    json_kv(std::cout, "wd_ready_valid", s.signals1.wd_ready_valid);
    json_kv(std::cout, "emergency_signal", s.signals1.emergency_signal);
    json_kv(std::cout, "emergency_signal_valid", s.signals1.emergency_signal_valid);
    json_kv(std::cout, "bms_dead", s.signals1.bms_dead);
    json_kv(std::cout, "bms_dead_valid", s.signals1.bms_dead_valid);
    json_kv(std::cout, "ebs_engage", s.signals1.ebs_engage);
    json_kv(std::cout, "ebs_engage_valid", s.signals1.ebs_engage_valid);
    json_kv(std::cout, "ebs_release", s.signals1.ebs_release);
    json_kv(std::cout, "ebs_release_valid", s.signals1.ebs_release_valid);
    json_kv(std::cout, "steer_dead", s.signals1.steer_dead);
    json_kv(std::cout, "steer_dead_valid", s.signals1.steer_dead_valid);
    json_kv(std::cout, "pc_dead", s.signals1.pc_dead);
    json_kv(std::cout, "pc_dead_valid", s.signals1.pc_dead_valid);
    json_kv(std::cout, "inversor_dead", s.signals1.inversor_dead);
    json_kv(std::cout, "inversor_dead_valid", s.signals1.inversor_dead_valid);
    json_kv(std::cout, "res_dead", s.signals1.res_dead);
    json_kv(std::cout, "res_dead_valid", s.signals1.res_dead_valid);
    json_kv(std::cout, "state_checkup", (int64_t)s.signals1.state_checkup);
    json_kv(std::cout, "state_checkup_valid", s.signals1.state_checkup_valid);
    json_kv(std::cout, "dc_voltage", (int64_t)s.signals1.dc_voltage);
    json_kv(std::cout, "dc_voltage_valid", s.signals1.dc_voltage_valid);
    json_kv(std::cout, "mission", (int64_t)s.signals1.mission);
    json_kv(std::cout, "mission_valid", s.signals1.mission_valid);
    json_kv(std::cout, "state", (int64_t)s.signals1.state);
    json_kv(std::cout, "state_valid", s.signals1.state_valid);

    // ==== Hydraulic ====
    json_kv(std::cout, "hydraulic_front_bar", (double)s.hydraulic.front_bar);
    json_kv(std::cout, "hydraulic_front_valid", s.hydraulic.front_valid);
    json_kv(std::cout, "hydraulic_rear_bar", (double)s.hydraulic.rear_bar);
    json_kv(std::cout, "hydraulic_rear_valid", s.hydraulic.rear_valid);

    // ==== Dash ====
    json_kv(std::cout, "dash_driving_state", (int64_t)s.dash.driving_state);
    json_kv(std::cout, "dash_driving_state_valid", s.dash.driving_state_valid);
    json_kv(std::cout, "dash_implausibility", s.dash.implausibility);
    json_kv(std::cout, "dash_implausibility_valid", s.dash.implausibility_valid);
    json_kv(std::cout, "dash_brake_pressure", (int64_t)s.dash.brake_pressure);
    json_kv(std::cout, "dash_brake_pressure_valid", s.dash.brake_pressure_valid);
    json_kv(std::cout, "apps_higher", (int64_t)s.dash.apps_higher);
    json_kv(std::cout, "apps_higher_valid", s.dash.apps_higher_valid);
    json_kv(std::cout, "apps_lower", (int64_t)s.dash.apps_lower);
    json_kv(std::cout, "apps_lower_valid", s.dash.apps_lower_valid);
    json_kv(std::cout, "rpm_fr_raw", (int64_t)s.dash.rpm_fr_raw);
    json_kv(std::cout, "rpm_fr_valid", s.dash.rpm_fr_valid);
    json_kv(std::cout, "rpm_fl_raw", (int64_t)s.dash.rpm_fl_raw);
    json_kv(std::cout, "rpm_fl_valid", s.dash.rpm_fl_valid);

    // ==== Master ====
    json_kv(std::cout, "master_asms_on", s.master.asms_on);
    json_kv(std::cout, "master_asms_on_valid", s.master.asms_on_valid);
    json_kv(std::cout, "master_soc", (int64_t)s.master.soc);
    json_kv(std::cout, "master_soc_valid", s.master.soc_valid);
    json_kv(std::cout, "master_as_state", (int64_t)s.master.as_state);
    json_kv(std::cout, "master_as_state_valid", s.master.as_state_valid);
    json_kv(std::cout, "master_autonomous_mission", (int64_t)s.master.autonomous_mission);
    json_kv(std::cout, "master_autonomous_mission_valid", s.master.autonomous_mission_valid);

    // ==== Bamocar ====
    json_kv(std::cout, "bamocar_ts_on", s.bamocar.ts_on);
    json_kv(std::cout, "bamocar_ts_on_valid", s.bamocar.ts_on_valid);
    json_kv(std::cout, "bamocar_speed", (int64_t)s.bamocar.speed);
    json_kv(std::cout, "bamocar_speed_valid", s.bamocar.speed_valid);
    json_kv(std::cout, "bamocar_motor_current", (int64_t)s.bamocar.motor_current);
    json_kv(std::cout, "bamocar_motor_current_valid", s.bamocar.motor_current_valid);
    json_kv(std::cout, "bamocar_error_bitmap", (int64_t)s.bamocar.error_bitmap);
    json_kv(std::cout, "bamocar_error_bitmap_valid", s.bamocar.error_bitmap_valid);
    json_kv(std::cout, "bamocar_warning_bitmap", (int64_t)s.bamocar.warning_bitmap);
    json_kv(std::cout, "bamocar_warning_bitmap_valid", s.bamocar.warning_bitmap_valid);

    // ==== BMS Thermistors ====
    json_kv(std::cout, "therm_min", (int64_t)s.bms_thermistors.min_temp);
    json_kv(std::cout, "therm_min_valid", s.bms_thermistors.min_temp_valid);
    json_kv(std::cout, "therm_max", (int64_t)s.bms_thermistors.max_temp);
    json_kv(std::cout, "therm_max_valid", s.bms_thermistors.max_temp_valid);
    json_kv(std::cout, "therm_avg", (int64_t)s.bms_thermistors.avg_temp);
    json_kv(std::cout, "therm_avg_valid", s.bms_thermistors.avg_temp_valid);

    // ==== Cells globals ====
    json_kv(std::cout, "cells_global_min", (int64_t)s.cells.global_min);
    json_kv(std::cout, "cells_global_min_valid", s.cells.global_min_valid);
    json_kv(std::cout, "cells_global_max", (int64_t)s.cells.global_max);
    json_kv(std::cout, "cells_global_max_valid", s.cells.global_max_valid, true);

    std::cout << "}\n";
}

int main(int argc, char** argv)
{
    install_signal_handlers();

    if (argc < 2)
    {
#ifdef _WIN32
        std::cerr << "Usage: " << argv[0] << " COM3\n";
#elif __APPLE__
        std::cerr << "Usage: " << argv[0] << " /dev/cu.usbserial-XXXX\n";
#else
        std::cerr << "Usage: " << argv[0] << " /dev/ttyUSB0\n";
#endif
        return 1;
    }

    sensors::Config cfg;
    cfg.device = argv[1];

    if (!sensors::begin(cfg))
    {
        std::cerr << "Failed to start CANdapter on " << cfg.device << "\n";
        return 1;
    }

    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    constexpr auto period = std::chrono::milliseconds(20); // ~50 Hz

    while (g_run.load(std::memory_order_relaxed))
    {
        sensors::tick();
        const auto s = sensors::read();
        print_snapshot_json(s);

        next += period;
        std::this_thread::sleep_until(next);
    }

    sensors::stop();
    return 0;
}
