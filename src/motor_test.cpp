#include "app_config.hpp"
#include "motor_client.hpp"
#include "serial_port.hpp"

#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Options {
    std::string config_path;
    int         monitor_timeout_ms = 200;
};

struct MonitorState {
    int period_ms = 250;
    bool has_last = false;
    MotorFeedback last_feedback;
    bool line_visible = false;
    std::string last_line;
};

void print_usage(const char* argv0)
{
    printf("Usage: %s <config.yaml> [--monitor-timeout-ms N]\n", argv0);
}

bool parse_args(int argc, char* argv[], Options& opts)
{
    if (argc < 2)
        return false;
    if (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")
        return false;
    opts.config_path = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--monitor-timeout-ms" && i + 1 < argc) {
            opts.monitor_timeout_ms = std::stoi(argv[++i]);
        } else if (arg == "-h" || arg == "--help") {
            return false;
        } else {
            fprintf(stderr, "Unknown argument: %s\n", arg.c_str());
            return false;
        }
    }
    return true;
}

void print_help()
{
    printf("Commands:\n");
    printf("  vel <rad_s>                 send velocity command\n");
    printf("  stop                        send zero velocity\n");
    printf("  pos <rad>                   send position command if configured\n");
    printf("  poscnt <count>              send 0x55 absolute position count\n");
    printf("  pid <kp> <ki> <kd>          send PID command if configured\n");
    printf("  read [timeout_ms]           send 0x0B and read 20-byte system status\n");
    printf("  read2f [timeout_ms]         read legacy 15-byte 0x2F feedback packet\n");
    printf("  readsys [timeout_ms]        alias of read\n");
    printf("  monitor on|off [period_ms]  print feedback packets in background\n");
    printf("  frame <cmd_hex> [bytes...]  build 0x3E frame with CRC and send it\n");
    printf("  raw <bytes...>              send exact bytes, e.g. raw 3E 00 01 54 02 00 00\n");
    printf("  set velcmd|poscmd|pidcmd|feedback <hex|-1>\n");
    printf("  set address|sequence <hex>\n");
    printf("  set strictaddr|consume|debug on|off\n");
    printf("  set zerodeg|counts|pidscale <value>\n");
    printf("  protocol                    print protocol settings\n");
    printf("  help                        show this help\n");
    printf("  quit                        exit; sends stop if motion was commanded\n");
}

int parse_int_token(const std::string& token)
{
    char* end = nullptr;
    const long value = std::strtol(token.c_str(), &end, 0);
    if (!end || *end != '\0')
        throw std::runtime_error("invalid integer: " + token);
    return static_cast<int>(value);
}

bool parse_bool_token(const std::string& token)
{
    std::string lower = token;
    for (char& ch : lower)
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    if (lower == "1" || lower == "true" || lower == "on" || lower == "yes")
        return true;
    if (lower == "0" || lower == "false" || lower == "off" || lower == "no")
        return false;
    throw std::runtime_error("invalid boolean: " + token);
}

uint8_t parse_byte_token(const std::string& token)
{
    const int base = (token.size() > 2 && token[0] == '0' &&
                      (token[1] == 'x' || token[1] == 'X')) ? 0 : 16;
    char* end = nullptr;
    const long value = std::strtol(token.c_str(), &end, base);
    if (!end || *end != '\0' || value < 0 || value > 255)
        throw std::runtime_error("invalid byte: " + token);
    return static_cast<uint8_t>(value);
}

std::vector<uint8_t> parse_bytes(std::istringstream& iss)
{
    std::vector<uint8_t> bytes;
    std::string token;
    while (iss >> token)
        bytes.push_back(parse_byte_token(token));
    return bytes;
}

void print_bytes(const std::vector<uint8_t>& bytes)
{
    std::ios old_state(nullptr);
    old_state.copyfmt(std::cout);
    for (size_t i = 0; i < bytes.size(); ++i) {
        if (i != 0)
            std::cout << ' ';
        std::cout << std::uppercase << std::hex << std::setw(2)
                  << std::setfill('0') << static_cast<int>(bytes[i]);
    }
    std::cout << '\n';
    std::cout.copyfmt(old_state);
}

void print_feedback(const MotorFeedback& fb)
{
    printf("[fb] abs=%.5f rad (%.2f deg raw=%d) multi=%.5f rad raw_multi=%d "
           "joint=%.5f rad vel=%.5f rad/s raw_vel=%d",
           fb.absolute_position_rad, fb.absolute_position_deg,
           fb.raw_angle_count, fb.multi_turn_position_rad, fb.raw_multi_turn_count,
           fb.joint_position_rad, fb.velocity_rad_s, fb.raw_velocity_count);
    if (fb.system_status_valid) {
        printf(" U=%.2fV I=%.2fA T=%.2fC fault=0x%02X state=%u",
               fb.bus_voltage_v, fb.phase_current_a, fb.temperature_c,
               static_cast<unsigned>(fb.fault_code),
               static_cast<unsigned>(fb.run_state));
    }
    printf("\n");
}

std::string feedback_to_string(const MotorFeedback& fb)
{
    char buf[256];
    int n = std::snprintf(
        buf, sizeof(buf),
        "[fb] abs=%.5f rad (%.2f deg raw=%d) multi=%.5f rad raw_multi=%d "
        "joint=%.5f rad vel=%.5f rad/s raw_vel=%d",
        fb.absolute_position_rad, fb.absolute_position_deg,
        fb.raw_angle_count, fb.multi_turn_position_rad, fb.raw_multi_turn_count,
        fb.joint_position_rad, fb.velocity_rad_s, fb.raw_velocity_count);
    std::string out(buf, n > 0 ? static_cast<size_t>(n) : 0);
    if (fb.system_status_valid) {
        char tail[128];
        n = std::snprintf(tail, sizeof(tail),
                          " U=%.2fV I=%.2fA T=%.2fC fault=0x%02X state=%u",
                          fb.bus_voltage_v, fb.phase_current_a, fb.temperature_c,
                          static_cast<unsigned>(fb.fault_code),
                          static_cast<unsigned>(fb.run_state));
        if (n > 0)
            out.append(tail, static_cast<size_t>(n));
    }
    return out;
}

void print_protocol(const MotorProtocolConfig& cfg)
{
    printf("protocol: seq=0x%02X addr=0x%02X feedback=0x%02X velcmd=0x%02X counts/rev=%.3f pid_scale=%.3f\n",
           static_cast<unsigned>(cfg.sequence),
           static_cast<unsigned>(cfg.address),
           cfg.feedback_command, cfg.velocity_command,
           cfg.position_counts_per_rev, cfg.pid_scale);
    printf("  strict_address=%s consume_command_response=%s debug_frames=%s\n",
           cfg.strict_address ? "true" : "false",
           cfg.consume_command_response ? "true" : "false",
           cfg.debug_frames ? "true" : "false");
    if (cfg.position_command >= 0)
        printf("  poscmd=0x%02X\n", cfg.position_command);
    else
        printf("  poscmd=disabled\n");
    if (cfg.pid_command >= 0)
        printf("  pidcmd=0x%02X\n", cfg.pid_command);
    else
        printf("  pidcmd=disabled\n");
}

void print_read_stats(const MotorReadStats& stats)
{
    if (stats.skipped_frames == 0 && stats.crc_errors == 0 &&
        stats.address_mismatches == 0 && stats.malformed_frames == 0) {
        return;
    }
    printf("[rx] skipped=%d crc_bad=%d addr_bad=%d malformed=%d last_skip=0x%02X len=%u\n",
           stats.skipped_frames, stats.crc_errors, stats.address_mismatches,
           stats.malformed_frames,
           static_cast<unsigned>(stats.last_skipped_command),
           static_cast<unsigned>(stats.last_skipped_payload_len));
}

bool feedback_changed_enough(const MotorFeedback& a, const MotorFeedback& b)
{
    constexpr double kAngleEps = 1e-4;
    constexpr double kVelEps = 1e-4;
    constexpr double kVoltEps = 1e-3;
    constexpr double kCurrentEps = 1e-3;
    constexpr double kTempEps = 1e-3;

    return std::fabs(a.joint_position_rad - b.joint_position_rad) > kAngleEps ||
           std::fabs(a.velocity_rad_s - b.velocity_rad_s) > kVelEps ||
           std::fabs(a.bus_voltage_v - b.bus_voltage_v) > kVoltEps ||
           std::fabs(a.phase_current_a - b.phase_current_a) > kCurrentEps ||
           std::fabs(a.temperature_c - b.temperature_c) > kTempEps ||
           a.fault_code != b.fault_code ||
           a.run_state != b.run_state ||
           a.system_status_valid != b.system_status_valid;
}

void clear_monitor_line(MonitorState& state)
{
    if (!state.line_visible)
        return;
    std::printf("\r%*s\r", static_cast<int>(state.last_line.size()), "");
    std::fflush(stdout);
    state.line_visible = false;
    state.last_line.clear();
}

void draw_monitor_line(MonitorState& state, const std::string& line)
{
    std::printf("\r%-*s", static_cast<int>(state.last_line.size()), "");
    std::printf("\r%s", line.c_str());
    std::fflush(stdout);
    state.line_visible = true;
    state.last_line = line;
}

} // namespace

int main(int argc, char* argv[])
{
    Options opts;
    if (!parse_args(argc, argv, opts)) {
        print_usage(argv[0]);
        return 1;
    }

    AppConfig app;
    try {
        app = load_app_config(opts.config_path);
    } catch (const std::exception& e) {
        fprintf(stderr, "Config error: %s\n", e.what());
        return 1;
    }

    SerialPort port(app.serial_port, app.baud_rate);
    if (!port.open())
        return 1;
    port.flush_input();

    MotorClient motor(port, app.motor);
    double encoder_zero_deg = app.encoder_zero_offset_deg;

    std::mutex port_mutex;
    std::mutex print_mutex;
    std::atomic<bool> running{true};
    std::atomic<bool> monitor_enabled{false};
    std::atomic<bool> commanded_motion{false};
    std::atomic<int> monitor_period_ms{250};
    MonitorState monitor_state;

    std::thread monitor([&] {
        while (running) {
            if (!monitor_enabled) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            MotorFeedback fb;
            bool ok = false;
            {
                std::lock_guard<std::mutex> lock(port_mutex);
                ok = motor.read_system_status(encoder_zero_deg, fb,
                                              opts.monitor_timeout_ms, true);
            }
            if (ok) {
                std::lock_guard<std::mutex> lock(print_mutex);
                if (!monitor_state.has_last ||
                    feedback_changed_enough(fb, monitor_state.last_feedback)) {
                    draw_monitor_line(monitor_state, feedback_to_string(fb));
                    monitor_state.has_last = true;
                }
                monitor_state.last_feedback = fb;
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(std::max(20, monitor_period_ms.load())));
        }
    });

    printf("Motor test opened %s @ %d baud.\n", app.serial_port.c_str(), app.baud_rate);
    print_help();
    print_protocol(motor.protocol());
    printf("monitor off (period=%dms)\n", monitor_period_ms.load());

    std::string line;
    while (running) {
        {
            std::lock_guard<std::mutex> lock(print_mutex);
            clear_monitor_line(monitor_state);
            std::cout << "motor> " << std::flush;
        }
        if (!std::getline(std::cin, line))
            break;
        if (line.empty())
            continue;

        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;
        for (char& ch : cmd)
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));

        try {
            if (cmd == "help" || cmd == "?") {
                print_help();
            } else if (cmd == "quit" || cmd == "exit") {
                running = false;
                break;
            } else if (cmd == "protocol") {
                print_protocol(motor.protocol());
            } else if (cmd == "vel") {
                double value = 0.0;
                if (!(iss >> value))
                    throw std::runtime_error("usage: vel <rad_s>");
                std::lock_guard<std::mutex> lock(port_mutex);
                if (motor.send_velocity_rad_s(value)) {
                    commanded_motion = true;
                    printf("sent velocity %.6f rad/s\n", value);
                    print_read_stats(motor.last_read_stats());
                }
            } else if (cmd == "stop") {
                std::lock_guard<std::mutex> lock(port_mutex);
                if (motor.send_velocity_rad_s(0.0)) {
                    printf("sent stop\n");
                    print_read_stats(motor.last_read_stats());
                }
            } else if (cmd == "pos") {
                double value = 0.0;
                if (!(iss >> value))
                    throw std::runtime_error("usage: pos <rad>");
                std::lock_guard<std::mutex> lock(port_mutex);
                if (motor.send_position_rad(value)) {
                    commanded_motion = true;
                    printf("sent position %.6f rad\n", value);
                }
            } else if (cmd == "poscnt") {
                unsigned long value = 0;
                if (!(iss >> value))
                    throw std::runtime_error("usage: poscnt <count>");
                std::lock_guard<std::mutex> lock(port_mutex);
                if (motor.send_position_counts(static_cast<uint32_t>(value))) {
                    commanded_motion = true;
                    printf("sent position count %lu\n", value);
                }
            } else if (cmd == "pid") {
                double kp = 0.0, ki = 0.0, kd = 0.0;
                if (!(iss >> kp >> ki >> kd))
                    throw std::runtime_error("usage: pid <kp> <ki> <kd>");
                std::lock_guard<std::mutex> lock(port_mutex);
                if (motor.send_pid(kp, ki, kd))
                    printf("sent pid %.6f %.6f %.6f\n", kp, ki, kd);
            } else if (cmd == "read") {
                int timeout_ms = 1000;
                iss >> timeout_ms;
                MotorFeedback fb;
                bool ok = false;
                {
                    std::lock_guard<std::mutex> lock(port_mutex);
                    ok = motor.read_system_status(encoder_zero_deg, fb, timeout_ms);
                }
                if (ok) {
                    clear_monitor_line(monitor_state);
                    print_feedback(fb);
                    print_read_stats(motor.last_read_stats());
                } else {
                    fprintf(stderr,
                            "read timeout/no response on 0x0B, check motor power/address/wiring\n");
                    print_read_stats(motor.last_read_stats());
                }
            } else if (cmd == "read2f") {
                int timeout_ms = 1000;
                iss >> timeout_ms;
                MotorFeedback fb;
                bool ok = false;
                {
                    std::lock_guard<std::mutex> lock(port_mutex);
                    ok = motor.read_feedback(encoder_zero_deg, fb, timeout_ms);
                }
                if (ok) {
                    clear_monitor_line(monitor_state);
                    print_feedback(fb);
                    print_read_stats(motor.last_read_stats());
                } else {
                    fprintf(stderr,
                            "read2f timeout/no legacy 0x2F feedback available\n");
                    print_read_stats(motor.last_read_stats());
                }
            } else if (cmd == "readsys") {
                int timeout_ms = 1000;
                iss >> timeout_ms;
                MotorFeedback fb;
                bool ok = false;
                {
                    std::lock_guard<std::mutex> lock(port_mutex);
                    ok = motor.read_system_status(encoder_zero_deg, fb, timeout_ms);
                }
                if (ok) {
                    clear_monitor_line(monitor_state);
                    print_feedback(fb);
                    print_read_stats(motor.last_read_stats());
                } else {
                    fprintf(stderr,
                            "readsys timeout/no response on 0x0B, check motor power/address/wiring\n");
                    print_read_stats(motor.last_read_stats());
                }
            } else if (cmd == "monitor") {
                std::string mode;
                if (!(iss >> mode))
                    throw std::runtime_error("usage: monitor on|off [period_ms]");
                for (char& ch : mode)
                    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
                int period_ms = monitor_period_ms.load();
                if (iss >> period_ms) {
                    if (period_ms < 20)
                        throw std::runtime_error("monitor period must be >= 20 ms");
                    monitor_period_ms = period_ms;
                }
                if (mode == "on") {
                    monitor_enabled = true;
                    monitor_state.has_last = false;
                    clear_monitor_line(monitor_state);
                } else if (mode == "off") {
                    monitor_enabled = false;
                    monitor_state.has_last = false;
                    clear_monitor_line(monitor_state);
                } else {
                    throw std::runtime_error("usage: monitor on|off [period_ms]");
                }
                printf("monitor %s (period=%dms)\n",
                       monitor_enabled ? "on" : "off",
                       monitor_period_ms.load());
            } else if (cmd == "frame") {
                std::string cmd_token;
                if (!(iss >> cmd_token))
                    throw std::runtime_error("usage: frame <cmd_hex> [bytes...]");
                const uint8_t command = parse_byte_token(cmd_token);
                const std::vector<uint8_t> payload = parse_bytes(iss);
                const std::vector<uint8_t> frame =
                    build_motor_frame(command, payload,
                                      motor.protocol().sequence,
                                      motor.protocol().address);
                {
                    std::lock_guard<std::mutex> lock(port_mutex);
                    motor.send_raw(frame);
                }
                std::cout << "sent frame: ";
                print_bytes(frame);
            } else if (cmd == "raw") {
                const std::vector<uint8_t> bytes = parse_bytes(iss);
                if (bytes.empty())
                    throw std::runtime_error("usage: raw <bytes...>");
                {
                    std::lock_guard<std::mutex> lock(port_mutex);
                    motor.send_raw(bytes);
                }
                std::cout << "sent raw: ";
                print_bytes(bytes);
            } else if (cmd == "set") {
                std::string name;
                std::string value;
                if (!(iss >> name >> value))
                    throw std::runtime_error("usage: set <name> <value>");
                for (char& ch : name)
                    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));

                if (name == "velcmd") {
                    motor.protocol().velocity_command = parse_int_token(value);
                } else if (name == "poscmd") {
                    motor.protocol().position_command = parse_int_token(value);
                } else if (name == "pidcmd") {
                    motor.protocol().pid_command = parse_int_token(value);
                } else if (name == "feedback") {
                    motor.protocol().feedback_command = parse_int_token(value);
                } else if (name == "address") {
                    motor.protocol().address =
                        static_cast<uint8_t>(parse_int_token(value) & 0xFF);
                } else if (name == "sequence") {
                    motor.protocol().sequence =
                        static_cast<uint8_t>(parse_int_token(value) & 0xFF);
                } else if (name == "strictaddr") {
                    motor.protocol().strict_address = parse_bool_token(value);
                } else if (name == "consume") {
                    motor.protocol().consume_command_response = parse_bool_token(value);
                } else if (name == "debug") {
                    motor.protocol().debug_frames = parse_bool_token(value);
                } else if (name == "zerodeg") {
                    encoder_zero_deg = std::stod(value);
                } else if (name == "counts") {
                    motor.protocol().position_counts_per_rev = std::stod(value);
                } else if (name == "pidscale") {
                    motor.protocol().pid_scale = std::stod(value);
                } else {
                    throw std::runtime_error("unknown setting: " + name);
                }
                print_protocol(motor.protocol());
                printf("encoder_zero_deg=%.6f\n", encoder_zero_deg);
            } else {
                fprintf(stderr, "Unknown command: %s\n", cmd.c_str());
            }
        } catch (const std::exception& e) {
            fprintf(stderr, "Error: %s\n", e.what());
        }
    }

    running = false;
    monitor_enabled = false;
    if (monitor.joinable())
        monitor.join();

    clear_monitor_line(monitor_state);

    if (commanded_motion) {
        std::lock_guard<std::mutex> lock(port_mutex);
        motor.send_velocity_rad_s(0.0);
        printf("sent stop on exit\n");
    }

    return 0;
}
