#include "can_car_listener.hpp"

#include <unordered_map>
#include <mutex>
#include <chrono>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cerrno>
#include <cstring>

#ifdef _WIN32
    #include <windows.h>
#else
    // POSIX (macOS / Linux)
    #include <termios.h>
    #include <fcntl.h>
    #include <unistd.h>
    #include <sys/select.h>
#endif

namespace sensors
{
    // ===== Frame SLCAN =====
    struct CandapterFrame
    {
        uint32_t id = 0;
        uint8_t dlc = 0;
        uint8_t data[8]{};
        uint32_t timestampMs = 0; // A1 16-bit ms
        bool hasTimestamp = false;
        bool isExtended = false;
        bool isRemote = false;
    };

    // ===== State =====
    static Config g_cfg{};
    static bool g_running = false;

#ifdef _WIN32
    static HANDLE g_hSerial = INVALID_HANDLE_VALUE;
#else
    static int g_fd = -1;
#endif

    static std::mutex g_mu;
    static Snapshot g_snap{};

    static uint64_t t_snap = 0;

    // ===== Cross-platform sleep =====
    static inline void sleep_ms(unsigned ms)
    {
#ifdef _WIN32
        Sleep(ms);
#else
        usleep(ms * 1000u);
#endif
    }

    // ===== Helpers =====
    static inline uint64_t now_us()
    {
        using namespace std::chrono;
        return (uint64_t)duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
    }

    static inline bool expired(uint64_t last_us)
    {
        if (g_cfg.signal_timeout_ms == 0)
            return false;
        if (last_us == 0)
            return true;
        uint64_t age_us = now_us() - last_us;
        return age_us > (uint64_t)g_cfg.signal_timeout_ms * 1000ULL;
    }

    static uint32_t hexToUint(const std::string &s)
    {
        uint32_t v = 0;
        std::stringstream ss;
        ss << std::hex << s;
        ss >> v;
        return v;
    }

    static inline uint16_t u16_be(const uint8_t *d)
    {
        return (uint16_t(d[0]) << 8) | uint16_t(d[1]);
    }
    static inline uint32_t u32_be(const uint8_t *d)
    {
        return (uint32_t(d[0]) << 24) |
               (uint32_t(d[1]) << 16) |
               (uint32_t(d[2]) << 8) |
               uint32_t(d[3]);
    }
    static inline uint16_t u16_le(const uint8_t *d)
    {
        return uint16_t(d[0]) | (uint16_t(d[1]) << 8);
    }

    static inline int32_t i32_le(const uint8_t *d)
    {
        return int32_t(uint32_t(d[0]) |
                       (uint32_t(d[1]) << 8) |
                       (uint32_t(d[2]) << 16) |
                       (uint32_t(d[3]) << 24));
    }

    static inline uint32_t u32_le(const uint8_t *d)
    {
        return (uint32_t(d[0])) |
               (uint32_t(d[1]) << 8) |
               (uint32_t(d[2]) << 16) |
               (uint32_t(d[3]) << 24);
    }

    static inline uint8_t allTempsCompleteMask()
    {
        // ALL_TEMPS_CHUNKS = 3 (18/6)
        return (ALL_TEMPS_CHUNKS >= 8) ? 0xFFu : uint8_t((1u << ALL_TEMPS_CHUNKS) - 1u);
    }

    static void recompute_cells_global_minmax_unlocked()
    {
        // Uses per-board min/max; only considers boards with board_valid[board]==true
        bool any = false;
        int8_t gmin = 127;
        int8_t gmax = -128;

        for (uint8_t b = 0; b < NUM_BOARDS; ++b)
        {
            if (!g_snap.cells.board_valid[b])
                continue;
            any = true;
            gmin = std::min(gmin, g_snap.cells.board_min[b]);
            gmax = std::max(gmax, g_snap.cells.board_max[b]);
        }

        if (any)
        {
            g_snap.cells.global_min = gmin;
            g_snap.cells.global_min_valid = true;
            g_snap.cells.global_max = gmax;
            g_snap.cells.global_max_valid = true;
        }
        else
        {
            g_snap.cells.global_min_valid = false;
            g_snap.cells.global_max_valid = false;
        }
    }

    static inline float f32_native_from_4bytes(const uint8_t *d)
    {
        float f;
        static_assert(sizeof(float) == 4, "float must be 4 bytes");
        std::memcpy(&f, d, 4);
        return f;
    }

    static void invalidate_all(Snapshot &s)
    {
        // ---- Signals1 ----
        s.signals1.placeholder_valid = false;
        s.signals1.asms_on_valid = false;
        s.signals1.asats_pressed_valid = false;
        s.signals1.ats_pressed_valid = false;
        s.signals1.tsms_sdc_closed_valid = false;
        s.signals1.master_sdc_closed_valid = false;
        s.signals1.ts_on_valid = false;
        s.signals1.wd_ready_valid = false;

        s.signals1.emergency_signal_valid = false;
        s.signals1.bms_dead_valid = false;
        s.signals1.ebs_engage_valid = false;
        s.signals1.ebs_release_valid = false;
        s.signals1.steer_dead_valid = false;
        s.signals1.pc_dead_valid = false;
        s.signals1.inversor_dead_valid = false;
        s.signals1.res_dead_valid = false;

        s.signals1.state_checkup_valid = false;
        s.signals1.pneu_p1_valid = false;
        s.signals1.pneu_p2_valid = false;
        s.signals1.pneu_p_valid = false;

        s.signals1.dc_voltage_valid = false;
        s.signals1.mission_valid = false;
        s.signals1.state_valid = false;

        // ---- Hydraulic ----
        s.hydraulic.front_valid = false;
        s.hydraulic.rear_valid = false;

        // ---- Dash ----
        s.dash.driving_state_valid = false;
        s.dash.implausibility_valid = false;
        s.dash.brake_pressure_valid = false;
        s.dash.apps_higher_valid = false;
        s.dash.apps_lower_valid = false;
        s.dash.rpm_fr_valid = false;
        s.dash.rpm_fl_valid = false;

        // ---- Master ----
        s.master.brake_pressure_valid = false;
        s.master.asms_on_valid = false;
        s.master.soc_valid = false;
        s.master.as_state_valid = false;
        s.master.autonomous_mission_valid = false;

        // ---- Bamocar ----
        s.bamocar.ts_on_valid = false;
        s.bamocar.speed_valid = false;
        s.bamocar.motor_current_valid = false;
        s.bamocar.error_bitmap_valid = false;
        s.bamocar.warning_bitmap_valid = false;
        s.bamocar.btb_ready_valid = false;
        s.bamocar.transmission_enabled_valid = false;

        // ---- BMS Thermistors ----
        s.bms_thermistors.module_number_valid = false;
        s.bms_thermistors.min_temp_valid = false;
        s.bms_thermistors.max_temp_valid = false;
        s.bms_thermistors.avg_temp_valid = false;
        s.bms_thermistors.number_of_thermistors_valid = false;

        // ---- Cells ----
        for (uint8_t b = 0; b < NUM_BOARDS; ++b)
        {
            s.cells.board_valid[b] = false;
            s.cells.all_valid[b] = false;
            s.cells.chunks_mask[b] = 0;
        }
        s.cells.global_min_valid = false;
        s.cells.global_max_valid = false;
        s.t_us = 0;
        t_snap = 0;
    }

    // =================================================================
    //  Serial port abstraction
    // =================================================================

#ifdef _WIN32
    // ---- Windows serial implementation ----

    static HANDLE openSerialPort(const std::string &device, int baud)
    {
        // Windows needs "\\\\.\\COM3" format for COM ports >= 10,
        // but it also works for COM1-COM9
        std::string portPath = device;
        if (device.rfind("COM", 0) == 0 || device.rfind("com", 0) == 0)
            portPath = "\\\\.\\" + device;

        HANDLE h = CreateFileA(
            portPath.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            nullptr,
            OPEN_EXISTING,
            0,
            nullptr
        );

        if (h == INVALID_HANDLE_VALUE)
        {
            std::cerr << "open serial: CreateFileA failed for " << device
                      << " (error " << GetLastError() << ")\n";
            return INVALID_HANDLE_VALUE;
        }

        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);

        if (!GetCommState(h, &dcb))
        {
            std::cerr << "GetCommState failed\n";
            CloseHandle(h);
            return INVALID_HANDLE_VALUE;
        }

        dcb.BaudRate = (DWORD)baud;
        dcb.ByteSize = 8;
        dcb.Parity   = NOPARITY;
        dcb.StopBits  = ONESTOPBIT;
        dcb.fBinary   = TRUE;
        dcb.fParity   = FALSE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl  = DTR_CONTROL_ENABLE;
        dcb.fRtsControl  = RTS_CONTROL_ENABLE;
        dcb.fOutX = FALSE;
        dcb.fInX  = FALSE;

        if (!SetCommState(h, &dcb))
        {
            std::cerr << "SetCommState failed\n";
            CloseHandle(h);
            return INVALID_HANDLE_VALUE;
        }

        // Non-blocking: return immediately with whatever is in the buffer
        COMMTIMEOUTS timeouts{};
        timeouts.ReadIntervalTimeout         = MAXDWORD;
        timeouts.ReadTotalTimeoutMultiplier   = 0;
        timeouts.ReadTotalTimeoutConstant     = 0;
        timeouts.WriteTotalTimeoutMultiplier  = 0;
        timeouts.WriteTotalTimeoutConstant    = 1000;

        if (!SetCommTimeouts(h, &timeouts))
        {
            std::cerr << "SetCommTimeouts failed\n";
            CloseHandle(h);
            return INVALID_HANDLE_VALUE;
        }

        PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
        return h;
    }

    static bool writeAll(const std::string &data)
    {
        DWORD written = 0;
        DWORD total = 0;
        DWORD len = (DWORD)data.size();

        while (total < len)
        {
            if (!WriteFile(g_hSerial, data.c_str() + total, len - total, &written, nullptr))
            {
                std::cerr << "WriteFile failed (error " << GetLastError() << ")\n";
                return false;
            }
            total += written;
        }
        return true;
    }

    static bool readLineNonBlocking(std::string &outLine)
    {
        static std::string buffer;

        auto try_pop_line = [&]() -> bool
        {
            while (true)
            {
                size_t p = buffer.find_first_of("\r\n");
                if (p == std::string::npos)
                    return false;

                outLine = buffer.substr(0, p);
                size_t k = p;
                while (k < buffer.size() && (buffer[k] == '\r' || buffer[k] == '\n'))
                    ++k;
                buffer.erase(0, k);

                if (!outLine.empty())
                    return true;
            }
        };
        if (try_pop_line())
            return true;

        char tmp[256];
        DWORD bytesRead = 0;

        if (!ReadFile(g_hSerial, tmp, sizeof(tmp), &bytesRead, nullptr))
            return false;

        if (bytesRead == 0)
            return false;

        buffer.append(tmp, bytesRead);

        return try_pop_line();
    }

#else
    // ---- POSIX (macOS / Linux) serial implementation ----

    static int openSerialPort(const std::string &device, int baud)
    {
        int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY);
        if (fd < 0)
        {
            std::perror("open");
            return -1;
        }

        termios tty{};
        if (tcgetattr(fd, &tty) != 0)
        {
            std::perror("tcgetattr");
            ::close(fd);
            return -1;
        }

        cfmakeraw(&tty);

        speed_t speed = B115200;
        (void)baud;
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= (CLOCAL | CREAD);

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
            std::perror("tcsetattr");
            ::close(fd);
            return -1;
        }

        tcflush(fd, TCIOFLUSH);
        return fd;
    }

    static bool writeAll(const std::string &data)
    {
        const char *buf = data.c_str();
        size_t total = 0;
        size_t len = data.size();

        while (total < len)
        {
            ssize_t n = ::write(g_fd, buf + total, len - total);
            if (n < 0)
            {
                if (errno == EINTR)
                    continue;
                std::perror("write");
                return false;
            }
            if (n == 0)
                break;
            total += (size_t)n;
        }
        return true;
    }

    static bool readLineNonBlocking(std::string &outLine)
    {
        static std::string buffer;

        auto try_pop_line = [&]() -> bool
        {
            while (true)
            {
                size_t p = buffer.find_first_of("\r\n");
                if (p == std::string::npos)
                    return false;

                outLine = buffer.substr(0, p);
                size_t k = p;
                while (k < buffer.size() && (buffer[k] == '\r' || buffer[k] == '\n'))
                    ++k;
                buffer.erase(0, k);

                if (!outLine.empty())
                    return true;
            }
        };
        if (try_pop_line())
            return true;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(g_fd, &rfds);

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 0;

        int ret = select(g_fd + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0)
        {
            if (errno == EINTR)
                return false;
            std::perror("select");
            return false;
        }
        if (ret == 0)
            return false;

        char tmp[256];
        ssize_t n = ::read(g_fd, tmp, sizeof(tmp));
        if (n < 0)
        {
            if (errno == EINTR)
                return false;
            std::perror("read");
            return false;
        }
        if (n == 0)
            return false;

        buffer.append(tmp, tmp + n);

        return try_pop_line();
    }
#endif

    // ===== Parser SLCAN =====
    static bool parseCandapterLine(const std::string &line, CandapterFrame &out)
    {
        if (line.empty())
            return false;

        char type = line[0];
        size_t pos = 1;

        bool extended = false;
        bool remote = false;

        uint32_t id = 0;
        uint8_t dlc = 0;

        auto read_dlc = [&](uint8_t &dlc_out, size_t &pos_io) -> bool
        {
            if (pos_io >= line.size())
                return false;
            char c = line[pos_io];
            if (c < '0' || c > '8')
                return false;
            dlc_out = static_cast<uint8_t>(c - '0');
            ++pos_io;
            return true;
        };

        if (type == 't')
        {
            if (line.size() < pos + 3 + 1)
                return false;
            id = hexToUint(line.substr(pos, 3));
            pos += 3;
            if (!read_dlc(dlc, pos)) return false;
        }
        else if (type == 'T' || type == 'x')
        {
            extended = true;
            if (line.size() < pos + 8 + 1)
                return false;
            id = hexToUint(line.substr(pos, 8));
            pos += 8;
            if (!read_dlc(dlc, pos)) return false;
        }
        else if (type == 'r')
        {
            remote = true;
            if (line.size() < pos + 3 + 1)
                return false;
            id = hexToUint(line.substr(pos, 3));
            pos += 3;
            if (!read_dlc(dlc, pos)) return false;
        }
        else if (type == 'R')
        {
            extended = true;
            remote = true;
            if (line.size() < pos + 8 + 1)
                return false;
            id = hexToUint(line.substr(pos, 8));
            pos += 8;
            if (!read_dlc(dlc, pos)) return false;
        }
        else
        {
            return false;
        }

        if (dlc > 8)
            return false;

        if (!remote)
        {
            size_t dataHexChars = (size_t)dlc * 2;
            if (line.size() < pos + dataHexChars)
                return false;

            for (uint8_t i = 0; i < dlc; ++i)
                out.data[i] = (uint8_t)hexToUint(line.substr(pos + (size_t)i * 2, 2));

            pos += dataHexChars;
        }

        // timestamp A1: last 4 hex chars (if present)
        uint32_t ts = 0;
        bool hasTs = false;
        if (line.size() == pos + 4)
        {
            ts = hexToUint(line.substr(line.size() - 4, 4));
            hasTs = true;
        }

        out.id = id;
        out.dlc = dlc;
        out.timestampMs = ts;
        out.hasTimestamp = hasTs;
        out.isExtended = extended;
        out.isRemote = remote;

        return true;
    }

    // ===== Frame -> Snapshot =====
    static void onFrame(const CandapterFrame &f)
    {
        const uint64_t t = now_us();
        std::lock_guard<std::mutex> lk(g_mu);

        bool updated = false;

        // =========================
        // DATA_LOGGER_SIGNALS_1 (0x511)
        // =========================
        if (f.id == g_cfg.DATA_LOGGER_SIGNALS_1 && f.dlc >= 8)
        {
            const uint8_t b0 = f.data[0];
            const uint8_t b1 = f.data[1];
            const uint8_t b2 = f.data[2];

            // byte0 (bits 7..0)
            g_snap.signals1.placeholder = (b0 & (1u << 7)) != 0;
            g_snap.signals1.placeholder_valid = true;
            g_snap.signals1.asms_on = (b0 & (1u << 6)) != 0;
            g_snap.signals1.asms_on_valid = true;
            g_snap.signals1.asats_pressed = (b0 & (1u << 5)) != 0;
            g_snap.signals1.asats_pressed_valid = true;
            g_snap.signals1.ats_pressed = (b0 & (1u << 4)) != 0;
            g_snap.signals1.ats_pressed_valid = true;
            g_snap.signals1.tsms_sdc_closed = (b0 & (1u << 3)) != 0;
            g_snap.signals1.tsms_sdc_closed_valid = true;
            g_snap.signals1.master_sdc_closed = (b0 & (1u << 2)) != 0;
            g_snap.signals1.master_sdc_closed_valid = true;
            g_snap.signals1.ts_on = (b0 & (1u << 1)) != 0;
            g_snap.signals1.ts_on_valid = true;
            g_snap.signals1.wd_ready = (b0 & (1u << 0)) != 0;
            g_snap.signals1.wd_ready_valid = true;

            // byte1 (bits 7..0)
            g_snap.signals1.emergency_signal = (b1 & (1u << 7)) != 0;
            g_snap.signals1.emergency_signal_valid = true;
            g_snap.signals1.bms_dead = (b1 & (1u << 6)) != 0;
            g_snap.signals1.bms_dead_valid = true;
            g_snap.signals1.ebs_engage = (b1 & (1u << 5)) != 0;
            g_snap.signals1.ebs_engage_valid = true;
            g_snap.signals1.ebs_release = (b1 & (1u << 4)) != 0;
            g_snap.signals1.ebs_release_valid = true;
            g_snap.signals1.steer_dead = (b1 & (1u << 3)) != 0;
            g_snap.signals1.steer_dead_valid = true;
            g_snap.signals1.pc_dead = (b1 & (1u << 2)) != 0;
            g_snap.signals1.pc_dead_valid = true;
            g_snap.signals1.inversor_dead = (b1 & (1u << 1)) != 0;
            g_snap.signals1.inversor_dead_valid = true;
            g_snap.signals1.res_dead = (b1 & (1u << 0)) != 0;
            g_snap.signals1.res_dead_valid = true;

            // byte2
            g_snap.signals1.state_checkup = (b2 & 0x0F);
            g_snap.signals1.state_checkup_valid = true;
            g_snap.signals1.pneu_p1 = (b2 & (1u << 7)) != 0;
            g_snap.signals1.pneu_p1_valid = true;
            g_snap.signals1.pneu_p2 = (b2 & (1u << 6)) != 0;
            g_snap.signals1.pneu_p2_valid = true;
            g_snap.signals1.pneu_p = (b2 & (1u << 5)) != 0;
            g_snap.signals1.pneu_p_valid = true;

            // dc_voltage bytes 3..6 (BE)
            g_snap.signals1.dc_voltage = u32_be(&f.data[3]);
            g_snap.signals1.dc_voltage_valid = true;

            // byte7: mission low nibble, state high nibble
            const uint8_t b7 = f.data[7];
            g_snap.signals1.mission = (b7 & 0x0F);
            g_snap.signals1.mission_valid = true;
            g_snap.signals1.state = (b7 >> 4) & 0x0F;
            g_snap.signals1.state_valid = true;

            updated = true;
        }

        // =========================
        // DASH_ID (0x132) mux on data[0]
        // =========================
        if (f.id == g_cfg.DASH_ID && f.dlc >= 1)
        {
            const uint8_t type = f.data[0];

            if (type == DRIVING_STATE && f.dlc >= 3)
            {
                g_snap.dash.driving_state = f.data[1];
                g_snap.dash.driving_state_valid = true;
                g_snap.dash.implausibility = (f.data[2] != 0);
                g_snap.dash.implausibility_valid = true;
                updated = true;
            }
            else if (type == HYDRAULIC_LINE && f.dlc >= 3)
            {
                // Dash sends LE: buf[1]=low, buf[2]=high
                g_snap.dash.brake_pressure = u16_le(&f.data[1]);
                g_snap.dash.brake_pressure_valid = true;
                updated = true;
            }
            else if (type == APPS_HIGHER && f.dlc >= 5)
            {
                g_snap.dash.apps_higher = i32_le(&f.data[1]);
                g_snap.dash.apps_higher_valid = true;
                updated = true;
            }
            else if (type == APPS_LOWER && f.dlc >= 5)
            {
                g_snap.dash.apps_lower = i32_le(&f.data[1]);
                g_snap.dash.apps_lower_valid = true;
                updated = true;
            }
            else if (type == FR_RPM && f.dlc >= 5)
            {
                // Store raw 32-bit payload until rpm_to_bytes() encoding is confirmed
                g_snap.dash.rpm_fr_raw = u32_le(&f.data[1]);
                g_snap.dash.rpm_fr_valid = true;
                updated = true;
            }
            else if (type == FL_RPM && f.dlc >= 5)
            {
                g_snap.dash.rpm_fl_raw = u32_le(&f.data[1]);
                g_snap.dash.rpm_fl_valid = true;
                updated = true;
            }
        }

        // =========================
        // MASTER_ID (0x300) mux on data[0]
        // =========================
        if (f.id == g_cfg.MASTER_ID && f.dlc >= 2)
        {
            const uint8_t type = f.data[0];

            if (type == HYDRAULIC_LINE && f.dlc >= 3)
            {
                g_snap.master.brake_pressure = u16_le(&f.data[1]);
                g_snap.master.brake_pressure_valid = true;
                updated = true;
            }
            else if (type == ASMS && f.dlc >= 2)
            {
                g_snap.master.asms_on = (f.data[1] != 0);
                g_snap.master.asms_on_valid = true;
                updated = true;
            }
            else if (type == SOC_MSG && f.dlc >= 2)
            {
                g_snap.master.soc = f.data[1];
                g_snap.master.soc_valid = true;
                updated = true;
            }
            else if (type == STATE_MSG && f.dlc >= 2)
            {
                g_snap.master.as_state = f.data[1];
                g_snap.master.as_state_valid = true;
                updated = true;
            }
            else if (type == MISSION_MSG && f.dlc >= 2)
            {
                g_snap.master.autonomous_mission = f.data[1];
                g_snap.master.autonomous_mission_valid = true;
                updated = true;
            }
        }

        // =========================
        // BAMO_RESPONSE_ID (0x181) mux on data[0] (RegID)
        // Teensy side expects either len=4 (16-bit) or len=6 (32-bit)
        // =========================
        if (f.id == g_cfg.BAMO_RESPONSE_ID && f.dlc >= 3)
        {
            const uint8_t reg = f.data[0];

            int32_t message_value = 0;
            if (f.dlc == 4)
            {
                // 16-bit data: value = (buf[2] << 8) | buf[1]
                message_value = (int32_t(uint32_t(f.data[2]) << 8) | uint32_t(f.data[1]));
            }
            else if (f.dlc >= 6)
            {
                // 32-bit data (matches Teensy logic): bytes [1..4] little-endian
                message_value = int32_t(uint32_t(f.data[1]) |
                                        (uint32_t(f.data[2]) << 8) |
                                        (uint32_t(f.data[3]) << 16) |
                                        (uint32_t(f.data[4]) << 24));
            }
            else
            {
                // Unsupported length for values; ignore safely
                message_value = 0;
            }

            if (reg == DC_VOLTAGE)
            {
                g_snap.bamocar.ts_on = (message_value >= int32_t(DC_THRESHOLD));
                g_snap.bamocar.ts_on_valid = true;
                updated = true;
            }
            else if (reg == SPEED_ACTUAL)
            {
                g_snap.bamocar.speed = message_value;
                g_snap.bamocar.speed_valid = true;
                updated = true;
            }
            else if (reg == CURRENT_ACTUAL)
            {
                g_snap.bamocar.motor_current = message_value;
                g_snap.bamocar.motor_current_valid = true;
                updated = true;
            }
            else if (reg == LOGICMAP_ERRORS)
            {
                const uint32_t u = uint32_t(message_value);
                g_snap.bamocar.error_bitmap = uint16_t(u & 0xFFFFu);
                g_snap.bamocar.warning_bitmap = uint16_t((u >> 16) & 0xFFFFu);
                g_snap.bamocar.error_bitmap_valid = true;
                g_snap.bamocar.warning_bitmap_valid = true;
                updated = true;
            }
            else if (reg == BTB_READY_0)
            {
                g_snap.bamocar.btb_ready = true;
                g_snap.bamocar.btb_ready_valid = true;
                updated = true;
            }
            else if (reg == ENABLE_0)
            {
                g_snap.bamocar.transmission_enabled = true;
                g_snap.bamocar.transmission_enabled_valid = true;
                updated = true;
            }
        }

        // =========================
        // BMS_THERMISTOR_ID (0x1839F380) EXTENDED
        // Payload: buf[0]=module, [1]=min, [2]=max, [3]=avg, [4]=count, ...
        // =========================
        if (f.id == g_cfg.BMS_THERMISTOR_ID && f.isExtended && f.dlc >= 4)
        {
            g_snap.bms_thermistors.module_number = f.data[0];
            g_snap.bms_thermistors.module_number_valid = true;

            g_snap.bms_thermistors.min_temp = int8_t(f.data[1]);
            g_snap.bms_thermistors.min_temp_valid = true;
            g_snap.bms_thermistors.max_temp = int8_t(f.data[2]);
            g_snap.bms_thermistors.max_temp_valid = true;
            g_snap.bms_thermistors.avg_temp = int8_t(f.data[3]);
            g_snap.bms_thermistors.avg_temp_valid = true;

            if (f.dlc >= 5)
            {
                g_snap.bms_thermistors.number_of_thermistors = f.data[4];
                g_snap.bms_thermistors.number_of_thermistors_valid = true;
            }

            updated = true;
        }

        // =========================
        // CELLS: per-board min/max/avg
        // ID = CELL_TEMPS_BASE_ID + board, len=4
        // buf[0]=board_id, buf[1]=min, buf[2]=max, buf[3]=avg
        // =========================
        if (f.id >= g_cfg.CELL_TEMPS_BASE_ID &&
            f.id < (g_cfg.CELL_TEMPS_BASE_ID + NUM_BOARDS) &&
            f.dlc == 4)
        {
            const uint8_t board_from_id = uint8_t(f.id - g_cfg.CELL_TEMPS_BASE_ID);
            const uint8_t board_from_buf = f.data[0];

            if (board_from_id == board_from_buf && board_from_id < NUM_BOARDS)
            {
                g_snap.cells.board_min[board_from_id] = int8_t(f.data[1]);
                g_snap.cells.board_max[board_from_id] = int8_t(f.data[2]);
                g_snap.cells.board_avg[board_from_id] = int8_t(f.data[3]);
                g_snap.cells.board_valid[board_from_id] = true;

                recompute_cells_global_minmax_unlocked();
                updated = true;
            }
        }

        // =========================
        // CELLS: all temps chunked
        // ID = ALL_TEMPS_ID + board
        // buf[0]=board_id, buf[1]=msg_index, then up to 6 temps (int8 packed in uint8)
        // =========================
        if (f.id >= g_cfg.ALL_TEMPS_ID &&
            f.id < (g_cfg.ALL_TEMPS_ID + NUM_BOARDS) &&
            f.dlc >= 2)
        {
            const uint8_t board_from_id = uint8_t(f.id - g_cfg.ALL_TEMPS_ID);
            const uint8_t board_from_buf = f.data[0];
            const uint8_t msg_index = f.data[1];

            if (board_from_id == board_from_buf &&
                board_from_id < NUM_BOARDS &&
                msg_index < ALL_TEMPS_CHUNKS)
            {
                const uint8_t temps_count = uint8_t(f.dlc - 2);
                const uint8_t start_sensor = uint8_t(msg_index * TEMPS_PER_CHUNK);

                for (uint8_t i = 0; i < temps_count; ++i)
                {
                    const uint8_t sensor = uint8_t(start_sensor + i);
                    if (sensor < NTC_SENSOR_COUNT)
                    {
                        // sent as uint8_t but represents signed temperature
                        g_snap.cells.all[board_from_id][sensor] = int8_t(f.data[2 + i]);
                    }
                }

                g_snap.cells.chunks_mask[board_from_id] |= uint8_t(1u << msg_index);

                if ((g_snap.cells.chunks_mask[board_from_id] & allTempsCompleteMask()) == allTempsCompleteMask())
                {
                    g_snap.cells.all_valid[board_from_id] = true;
                }

                updated = true;
            }
        }

        // =========================
        // Final: single timestamp update
        // =========================
        if (updated)
        {
            g_snap.t_us = t;
            t_snap = t;
        }
    }

    // ===== Public API =====
    bool begin(const Config &cfg)
    {
        g_cfg = cfg;

#ifdef _WIN32
        g_hSerial = openSerialPort(g_cfg.device, g_cfg.baud);
        if (g_hSerial == INVALID_HANDLE_VALUE)
            return false;
#else
        g_fd = openSerialPort(g_cfg.device, g_cfg.baud);
        if (g_fd < 0)
            return false;
#endif

        // init CANdapter (classic sequence)
        // C = close, Sx = speed, A1 = timestamps, O = open
        if (!writeAll("C\r"))
        {
            stop();
            return false;
        }
        sleep_ms(20);

        if (!g_cfg.slcan_speed_cmd.empty())
        {
            if (!writeAll(g_cfg.slcan_speed_cmd))
            {
                stop();
                return false;
            }
            sleep_ms(20);
        }

        if (g_cfg.enable_a1_timestamp)
        {
            if (!writeAll("A1\r"))
            {
                stop();
                return false;
            }
            sleep_ms(20);
        }

        if (!writeAll("O\r"))
        {
            stop();
            return false;
        }
        sleep_ms(20);

        g_running = true;
        return true;
    }

    void tick()
    {
        if (!g_running)
            return;

#ifdef _WIN32
        if (g_hSerial == INVALID_HANDLE_VALUE)
            return;
#else
        if (g_fd < 0)
            return;
#endif

        for (;;)
        {
            std::string line;
            if (!readLineNonBlocking(line))
                break;

            CandapterFrame f;
            if (!parseCandapterLine(line, f))
                continue;
            if (f.isRemote)
                continue;

            onFrame(f);
        }

        std::lock_guard<std::mutex> lk(g_mu);
        if (expired(t_snap))
        {
            invalidate_all(g_snap);
        }
    }

    Snapshot read()
    {
        std::lock_guard<std::mutex> lk(g_mu);
        return g_snap;
    }

    void stop()
    {
        g_running = false;

#ifdef _WIN32
        if (g_hSerial != INVALID_HANDLE_VALUE)
        {
            writeAll("C\r");
            sleep_ms(20);
            CloseHandle(g_hSerial);
            g_hSerial = INVALID_HANDLE_VALUE;
        }
#else
        if (g_fd >= 0)
        {
            writeAll("C\r");
            sleep_ms(20);
            ::close(g_fd);
            g_fd = -1;
        }
#endif
    }
}
