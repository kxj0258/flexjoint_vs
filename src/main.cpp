#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <ctime>
#include <exception>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <vector>

#if __has_include(<filesystem>)
#include <filesystem>
#define FLEXJOINT_HAVE_STD_FILESYSTEM 1
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
#define FLEXJOINT_HAVE_EXPERIMENTAL_FILESYSTEM 1
#else
#error "C++17 filesystem support is required"
#endif

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "app_config.hpp"
#include "serial_port.hpp"
#include "controller.hpp"
#include "motor_client.hpp"
#include "vision.hpp"

static std::atomic<bool> g_running{true};

static void sig_handler(int) { g_running = false; }

namespace {

#if defined(FLEXJOINT_HAVE_STD_FILESYSTEM)
namespace fs = std::filesystem;
#else
namespace fs = std::experimental::filesystem;
#endif
using Clock = std::chrono::steady_clock;
using WallClock = std::chrono::system_clock;

constexpr double kPi = 3.14159265358979323846;
constexpr int kFeedbackPollEvery = 3;
constexpr int kFeedbackTimeoutMs = 200;
constexpr int kFeedbackWarnEvery = 30;
constexpr float kInitialAngleWarnRad = 0.25f;

enum class ExitReason {
    Other,
    TaskCompleted,
    CtrlC,
    VisionFailures,
    FeedbackFailures,
    MaxRuntime,
    MaxControlCycles
};

enum class ControllerMode {
    Proposed,
    ProposedNoFast,
    BaselinePd,
    BaselinePdNoFast
};

struct VideoRecorder {
    bool enabled = false;
    bool attempted = false;
    bool ok = false;
    bool warning_reported = false;
    int frames = 0;
    cv::Size frame_size;
    fs::path path;
    cv::VideoWriter writer;
    std::string status = "disabled";
};

struct RunSummary {
    WallClock::time_point start_wall;
    WallClock::time_point end_wall;
    Clock::time_point start_steady;
    Clock::time_point end_steady;
    fs::path log_dir;
    fs::path data_file_path;
    fs::path config_copy_path;
    fs::path summary_path;
    fs::path raw_video_path;
    fs::path annotated_video_path;
    bool config_copy_ok = false;
    std::string config_copy_error;
    ExitReason exit_reason = ExitReason::Other;
    int total_frames = 0;
    double last_joint_angle_rad = 0.0;
    double last_joint_velocity_rad_s = 0.0;
    std::array<float, 6> last_img = {};
    double last_velocity_command_rad_s = 0.0;
    double last_joint_cal_rad_s = 0.0;
    bool last_feedback_ok = false;
    bool last_vision_ok = false;
    bool safety_limit_triggered = false;
    std::string last_safety_stop_reason = "none";
    bool first_zero_velocity_sent = false;
    bool final_zero_velocity_sent = false;
    bool return_zero_enabled = false;
    bool return_zero_command_sent = false;
    bool return_zero_reached = false;
    uint32_t return_zero_target_count = 0;
    double return_zero_target_single_turn_deg = 0.0;
    std::string return_zero_mode;
    std::vector<std::string> warnings;
};

ControllerMode parse_controller_mode(const std::string& mode)
{
    if (mode == "proposed")
        return ControllerMode::Proposed;
    if (mode == "proposed_no_fast")
        return ControllerMode::ProposedNoFast;
    if (mode == "baseline_pd")
        return ControllerMode::BaselinePd;
    if (mode == "baseline_pd_no_fast")
        return ControllerMode::BaselinePdNoFast;
    throw std::runtime_error(
        "unsupported experiment.controller_mode '" + mode +
        "'; expected proposed, proposed_no_fast, baseline_pd, or baseline_pd_no_fast");
}

const char* controller_mode_name(ControllerMode mode)
{
    switch (mode) {
    case ControllerMode::Proposed:
        return "proposed";
    case ControllerMode::ProposedNoFast:
        return "proposed_no_fast";
    case ControllerMode::BaselinePd:
        return "baseline_pd";
    case ControllerMode::BaselinePdNoFast:
        return "baseline_pd_no_fast";
    }
    return "proposed";
}

bool controller_mode_disables_fast(ControllerMode mode)
{
    return mode == ControllerMode::ProposedNoFast ||
           mode == ControllerMode::BaselinePdNoFast;
}

bool controller_mode_uses_baseline_pd(ControllerMode mode)
{
    return mode == ControllerMode::BaselinePd ||
           mode == ControllerMode::BaselinePdNoFast;
}

void run_controller_step(ControllerMode mode,
                         const ControlParams& base_params,
                         const float joint_state[2],
                         const float img_pos[6],
                         const float obs[4],
                         const float para_slow[9],
                         float q_c,
                         const float ydif[6],
                         float* joint_cal,
                         float para_update[17])
{
    ControlParams params = base_params;
    if (controller_mode_disables_fast(mode))
        params.K4 = 0.0f;

    if (!controller_mode_uses_baseline_pd(mode)) {
        cal_joint_vel(joint_state, img_pos, obs, para_slow, q_c,
                      joint_cal, para_update, params);
        return;
    }

    float baseline_update[8] = {};
    cal_control(joint_state, img_pos, obs, q_c, ydif,
                joint_cal, baseline_update, params);

    for (int i = 0; i < 9; ++i)
        para_update[i] = para_slow[i];
    para_update[9] = baseline_update[0];
    para_update[10] = baseline_update[1];
    para_update[11] = baseline_update[2];
    para_update[12] = baseline_update[3];
    para_update[13] = baseline_update[4];
    para_update[14] = baseline_update[5];
    para_update[15] = baseline_update[6];
    para_update[16] = baseline_update[7];
}

bool feature_points_near_boundary(const float img_pos[6], float width, float height,
                                  float margin)
{
    for (int i = 0; i < 3; ++i) {
        const float u = img_pos[2 * i];
        const float v = img_pos[2 * i + 1];
        if (u < margin || u > width - margin ||
            v < margin || v > height - margin) {
            return true;
        }
    }
    return false;
}

bool read_configured_feedback(MotorClient& motor, const MotorProtocolConfig& motor_cfg,
                              double encoder_zero_deg, MotorFeedback& feedback,
                              int timeout_ms, bool quiet)
{
    if (motor_cfg.feedback_command == 0x0B) {
        return motor.read_system_status(encoder_zero_deg, feedback, timeout_ms, quiet);
    }
    if (motor_cfg.feedback_command == 0x2F) {
        return motor.read_feedback(encoder_zero_deg, feedback, timeout_ms, quiet);
    }
    if (motor_cfg.feedback_command >= 0 && motor_cfg.feedback_command <= 0xFF) {
        return motor.read_feedback_command(
            static_cast<uint8_t>(motor_cfg.feedback_command),
            encoder_zero_deg, feedback, timeout_ms, quiet);
    }
    return false;
}

void warn(RunSummary& summary, const std::string& message)
{
    summary.warnings.push_back(message);
    fprintf(stderr, "Warning: %s\n", message.c_str());
}

std::tm local_time(std::time_t t)
{
    std::tm tm_value = {};
#ifdef _WIN32
    localtime_s(&tm_value, &t);
#else
    localtime_r(&t, &tm_value);
#endif
    return tm_value;
}

std::string format_wall_time(const WallClock::time_point& tp)
{
    const std::time_t t = WallClock::to_time_t(tp);
    const std::tm tm_value = local_time(t);
    std::ostringstream oss;
    oss << std::put_time(&tm_value, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

std::string timestamp_for_directory(const WallClock::time_point& tp)
{
    const std::time_t t = WallClock::to_time_t(tp);
    const std::tm tm_value = local_time(t);
    std::ostringstream oss;
    oss << std::put_time(&tm_value, "%Y%m%d_%H%M%S");
    return oss.str();
}

double unix_time_seconds(const WallClock::time_point& tp)
{
    return std::chrono::duration<double>(tp.time_since_epoch()).count();
}

fs::path weakly_canonical_or_absolute(const fs::path& path)
{
    try {
        if (path.is_absolute())
            return path;
        return fs::current_path() / path;
    } catch (...) {
        return path;
    }
}

fs::path find_project_dir(const fs::path& config_path)
{
    const fs::path abs_config = weakly_canonical_or_absolute(config_path);
    const fs::path config_parent = abs_config.parent_path();
    if (config_parent.filename() == "config")
        return config_parent.parent_path();

    std::error_code ec;
    fs::path current = fs::current_path(ec);
    if (ec)
        return config_parent.empty() ? fs::path(".") : config_parent;

    current = weakly_canonical_or_absolute(current);
    while (!current.empty()) {
        if (fs::exists(current / "CMakeLists.txt") &&
            fs::exists(current / "src") &&
            fs::exists(current / "config")) {
            return current;
        }
        if (current == current.root_path())
            break;
        current = current.parent_path();
    }
    return config_parent.empty() ? fs::current_path() : config_parent;
}

fs::path resolve_under_project(const fs::path& project_dir, const std::string& value)
{
    fs::path path(value);
    if (path.is_absolute())
        return path;
    return project_dir / path;
}

bool create_directory_checked(const fs::path& path, std::string& error)
{
    std::error_code ec;
    fs::create_directories(path, ec);
    if (ec) {
        error = ec.message();
        return false;
    }
    return true;
}

fs::path make_run_log_dir(const fs::path& root_dir,
                          const WallClock::time_point& start_wall)
{
    const std::string stamp = timestamp_for_directory(start_wall);
    for (int i = 0; i < 100; ++i) {
        std::ostringstream suffix;
        suffix << stamp;
        if (i > 0)
            suffix << "_" << std::setw(2) << std::setfill('0') << i;
        fs::path candidate = root_dir / suffix.str();
        std::error_code ec;
        if (fs::create_directories(candidate, ec))
            return candidate;
        if (ec)
            throw std::runtime_error("cannot create log directory " +
                                     candidate.string() + ": " + ec.message());
    }
    throw std::runtime_error("cannot create unique log directory under " +
                             root_dir.string());
}

void write_data_header(std::ofstream& out)
{
    const std::array<std::string, 26> state_names = {
        "state_joint_angle_rad",
        "state_joint_velocity_rad_s",
        "state_img_u1",
        "state_img_v1",
        "state_img_u2",
        "state_img_v2",
        "state_img_u3",
        "state_img_v3",
        "state_theta_0",
        "state_theta_1",
        "state_theta_2",
        "state_theta_3",
        "state_rho_0",
        "state_rho_1",
        "state_rho_2",
        "state_rho_3",
        "state_rho_4",
        "state_obs_0",
        "state_obs_1",
        "state_obs_2",
        "state_obs_3",
        "state_qc",
        "state_velocity_command_rad_s",
        "state_tau",
        "state_tau_s",
        "state_tau_f_c"
    };

    std::vector<std::string> columns = {
        "frame_index",
        "elapsed_time_s",
        "unix_time_s",
        "joint_angle_rad",
        "joint_velocity_rad_s",
        "img_u1",
        "img_v1",
        "img_u2",
        "img_v2",
        "img_u3",
        "img_v3"
    };
    columns.insert(columns.end(), state_names.begin(), state_names.end());
    columns.push_back("velocity_command_rad_s");
    columns.push_back("joint_cal_rad_s");
    columns.push_back("feedback_ok");
    columns.push_back("vision_ok");
    columns.push_back("safety_stop_reason");

    for (size_t i = 0; i < columns.size(); ++i) {
        if (i > 0)
            out << ',';
        out << columns[i];
    }
    out << '\n';
}

void write_data_row(std::ofstream& out, int frame_index, double elapsed_s,
                    double unix_s, const float state[26], const float img_pos[6],
                    double velocity_cmd, double joint_cal, bool feedback_ok,
                    bool vision_ok, const std::string& safety_stop_reason)
{
    out << std::fixed << std::setprecision(6);
    out << frame_index << ','
        << elapsed_s << ','
        << unix_s << ','
        << state[0] << ','
        << state[1];
    for (int i = 0; i < 6; ++i)
        out << ',' << img_pos[i];
    for (int i = 0; i < 26; ++i)
        out << ',' << state[i];
    out << ',' << velocity_cmd
        << ',' << joint_cal
        << ',' << (feedback_ok ? 1 : 0)
        << ',' << (vision_ok ? 1 : 0)
        << ',' << (safety_stop_reason.empty() ? "none" : safety_stop_reason)
        << '\n';
}

std::string exit_reason_name(ExitReason reason)
{
    switch (reason) {
    case ExitReason::TaskCompleted:
        return "task_completed";
    case ExitReason::CtrlC:
        return "ctrl_c";
    case ExitReason::VisionFailures:
        return "vision_failures";
    case ExitReason::FeedbackFailures:
        return "feedback_failures";
    case ExitReason::MaxRuntime:
        return "max_runtime";
    case ExitReason::MaxControlCycles:
        return "max_control_cycles";
    case ExitReason::Other:
    default:
        return "other";
    }
}

void append_reason(std::string& current, const std::string& reason)
{
    if (current.empty() || current == "none") {
        current = reason;
    } else {
        current += "+";
        current += reason;
    }
}

double image_rms_error(const float img_pos[6], const float desired[6])
{
    double sum_sq = 0.0;
    for (int i = 0; i < 3; ++i) {
        const double du = static_cast<double>(img_pos[2 * i] - desired[2 * i]);
        const double dv = static_cast<double>(img_pos[2 * i + 1] - desired[2 * i + 1]);
        sum_sq += du * du + dv * dv;
    }
    return std::sqrt(sum_sq / 3.0);
}

uint32_t fourcc_from_string(const std::string& codec)
{
    std::string c = codec;
    if (c.size() < 4)
        c.append(4 - c.size(), ' ');
    return static_cast<uint32_t>(
        cv::VideoWriter::fourcc(c[0], c[1], c[2], c[3]));
}

bool open_video_writer(VideoRecorder& recorder, double fps,
                       const std::string& codec, RunSummary& summary)
{
    if (!recorder.enabled)
        return false;
    if (recorder.attempted)
        return recorder.ok;

    recorder.attempted = true;
    std::string error;
    if (!create_directory_checked(recorder.path.parent_path(), error)) {
        recorder.status = "open_failed";
        warn(summary, "cannot create video directory for " +
                      recorder.path.string() + ": " + error);
        return false;
    }

    const double writer_fps = fps > 0.0 ? fps : 30.0;
    try {
        recorder.writer.open(recorder.path.string(),
                             static_cast<int>(fourcc_from_string(codec)),
                             writer_fps, recorder.frame_size, true);
    } catch (const cv::Exception& e) {
        recorder.status = "open_failed";
        warn(summary, "cannot open video writer for " +
                      recorder.path.string() + ": " + e.what());
        return false;
    }

    recorder.ok = recorder.writer.isOpened();
    recorder.status = recorder.ok ? "recording" : "open_failed";
    if (!recorder.ok) {
        warn(summary, "cannot open video writer for " + recorder.path.string());
        return false;
    }

    printf("Video recording started: %s %dx%d @ %.2f fps\n",
           recorder.path.string().c_str(), recorder.frame_size.width,
           recorder.frame_size.height, writer_fps);
    return true;
}

void write_video_frame(VideoRecorder& recorder, const cv::Mat& frame, double fps,
                       const std::string& codec, RunSummary& summary)
{
    if (!recorder.enabled || frame.empty())
        return;

    if (recorder.frame_size.width <= 0 || recorder.frame_size.height <= 0)
        recorder.frame_size = frame.size();

    if (!open_video_writer(recorder, fps, codec, summary))
        return;

    try {
        cv::Mat output;
        if (frame.channels() == 1) {
            cv::cvtColor(frame, output, cv::COLOR_GRAY2BGR);
        } else {
            output = frame;
        }

        if (output.size() != recorder.frame_size) {
            cv::Mat resized;
            cv::resize(output, resized, recorder.frame_size);
            output = resized;
            if (!recorder.warning_reported) {
                recorder.warning_reported = true;
                warn(summary, "video frame size changed for " +
                              recorder.path.string() + "; frame was resized");
            }
        }

        recorder.writer.write(output);
        ++recorder.frames;
    } catch (const cv::Exception& e) {
        recorder.status = "write_failed";
        if (!recorder.warning_reported) {
            recorder.warning_reported = true;
            warn(summary, "video write failed for " +
                          recorder.path.string() + ": " + e.what());
        }
    }
}

void close_video_writer(VideoRecorder& recorder)
{
    if (recorder.writer.isOpened())
        recorder.writer.release();
    if (recorder.enabled && recorder.ok && recorder.status == "recording")
        recorder.status = "closed";
}

bool joint_position_to_single_turn_count(double encoder_zero_deg,
                                         double joint_position_rad,
                                         double counts_per_rev,
                                         uint32_t& target_count,
                                         double& target_single_turn_deg)
{
    if (counts_per_rev <= 0.0 ||
        counts_per_rev > static_cast<double>(std::numeric_limits<uint32_t>::max())) {
        return false;
    }

    target_single_turn_deg =
        encoder_zero_deg + joint_position_rad * 180.0 / kPi;
    double normalized_deg = std::fmod(target_single_turn_deg, 360.0);
    if (normalized_deg < 0.0)
        normalized_deg += 360.0;

    const long long counts_per_rev_i =
        std::max(1LL, static_cast<long long>(std::llround(counts_per_rev)));
    long long count = static_cast<long long>(
        std::llround(normalized_deg / 360.0 * counts_per_rev));
    count %= counts_per_rev_i;
    if (count < 0)
        count += counts_per_rev_i;
    target_count = static_cast<uint32_t>(count);
    return true;
}

bool motor_position_rad_to_single_turn_count(double motor_position_rad,
                                             double counts_per_rev,
                                             uint32_t& target_count,
                                             double& target_single_turn_deg)
{
    if (counts_per_rev <= 0.0 ||
        counts_per_rev > static_cast<double>(std::numeric_limits<uint32_t>::max())) {
        return false;
    }

    target_single_turn_deg = motor_position_rad * 180.0 / kPi;
    double normalized_rad = std::fmod(motor_position_rad, 2.0 * kPi);
    if (normalized_rad < 0.0)
        normalized_rad += 2.0 * kPi;

    const long long counts_per_rev_i =
        std::max(1LL, static_cast<long long>(std::llround(counts_per_rev)));
    long long count = static_cast<long long>(
        std::llround(normalized_rad / (2.0 * kPi) * counts_per_rev));
    count %= counts_per_rev_i;
    if (count < 0)
        count += counts_per_rev_i;
    target_count = static_cast<uint32_t>(count);
    return true;
}

void write_summary_file(const RunSummary& summary, const AppConfig& cfg,
                        const VideoRecorder& raw_video,
                        const VideoRecorder& annotated_video)
{
    std::ofstream out(summary.summary_path.string());
    if (!out.is_open()) {
        fprintf(stderr, "Warning: cannot open run summary: %s\n",
                summary.summary_path.string().c_str());
        return;
    }

    const double duration_s =
        std::chrono::duration<double>(summary.end_steady - summary.start_steady).count();

    out << "# flexjoint_vs Run Summary\n\n";
    out << "- Start time: " << format_wall_time(summary.start_wall) << "\n";
    out << "- End time: " << format_wall_time(summary.end_wall) << "\n";
    out << "- Duration: " << std::fixed << std::setprecision(3)
        << duration_s << " s\n";
    out << "- Total frames: " << summary.total_frames << "\n";
    out << "- Exit reason: " << exit_reason_name(summary.exit_reason) << "\n";
    out << "- Last joint angle: " << summary.last_joint_angle_rad << " rad\n";
    out << "- Last joint velocity: " << summary.last_joint_velocity_rad_s
        << " rad/s\n";
    out << "- Last image points: [";
    for (int i = 0; i < 6; ++i) {
        if (i > 0)
            out << ", ";
        out << summary.last_img[i];
    }
    out << "]\n";
    out << "- Last velocity command: "
        << summary.last_velocity_command_rad_s << " rad/s\n";
    out << "- Last joint_cal: " << summary.last_joint_cal_rad_s << " rad/s\n";
    out << "- Last feedback ok: " << (summary.last_feedback_ok ? "true" : "false") << "\n";
    out << "- Last vision ok: " << (summary.last_vision_ok ? "true" : "false") << "\n";
    out << "- Safety limit triggered: "
        << (summary.safety_limit_triggered ? "true" : "false") << "\n";
    out << "- Last safety stop reason: "
        << summary.last_safety_stop_reason << "\n\n";

    out << "## Files\n\n";
    out << "- Log directory: " << summary.log_dir.string() << "\n";
    out << "- Data log: " << summary.data_file_path.string() << "\n";
    out << "- Run config: " << summary.config_copy_path.string()
        << " (" << (summary.config_copy_ok ? "copied" : "copy failed") << ")\n";
    if (!summary.config_copy_error.empty())
        out << "- Run config copy error: " << summary.config_copy_error << "\n";
    out << "- Raw video: " << raw_video.path.string()
        << " (" << raw_video.status << ", frames=" << raw_video.frames << ")\n";
    out << "- Annotated video: " << annotated_video.path.string()
        << " (" << annotated_video.status << ", frames="
        << annotated_video.frames << ")\n";
    out << "- Summary: " << summary.summary_path.string() << "\n\n";

    out << "## Cleanup\n\n";
    out << "- First zero velocity sent: "
        << (summary.first_zero_velocity_sent ? "true" : "false") << "\n";
    out << "- Return to zero enabled: "
        << (summary.return_zero_enabled ? "true" : "false") << "\n";
    out << "- Return zero mode: " << summary.return_zero_mode << "\n";
    out << "- Return zero joint target: "
        << cfg.task.return_zero_joint_rad << " rad\n";
    out << "- Return zero motor position target: "
        << cfg.task.return_zero_motor_pos_rad << " rad\n";
    out << "- Return zero single-turn target: "
        << summary.return_zero_target_single_turn_deg << " deg\n";
    out << "- Return zero target count: "
        << summary.return_zero_target_count << "\n";
    out << "- Return zero command sent: "
        << (summary.return_zero_command_sent ? "true" : "false") << "\n";
    out << "- Return zero reached before timeout: "
        << (summary.return_zero_reached ? "true" : "false") << "\n";
    out << "- Final zero velocity sent: "
        << (summary.final_zero_velocity_sent ? "true" : "false") << "\n\n";

    out << "## Main Config\n\n";
    out << "- experiment.controller_mode: "
        << cfg.experiment.controller_mode << "\n";
    out << "- desired_coords: [";
    for (int i = 0; i < 6; ++i) {
        if (i > 0)
            out << ", ";
        out << cfg.ctrl.yd[i];
    }
    out << "]\n";
    out << "- initial_angle_rad: " << cfg.initial_angle_rad << "\n";
    out << "- encoder_zero_offset_deg: " << cfg.encoder_zero_offset_deg << "\n";
    out << "- safe_angle_range_rad: [" << cfg.task.min_safe_angle_rad
        << ", " << cfg.task.max_safe_angle_rad << "]\n";
    out << "- velocity_saturation_rad_s: "
        << cfg.task.velocity_saturation_rad_s << "\n";
    out << "- completion_check: "
        << (cfg.task.enable_completion_check ? "true" : "false")
        << ", image_error_tolerance_px=" << cfg.task.image_error_tolerance_px
        << ", velocity_tolerance_rad_s=" << cfg.task.velocity_tolerance_rad_s
        << ", stable_frames_required=" << cfg.task.stable_frames_required
        << ", max_runtime_s=" << cfg.task.max_runtime_s
        << ", max_control_cycles=" << cfg.task.max_control_cycles << "\n";
    out << "- return_zero: mode=" << cfg.task.return_zero_mode
        << ", joint_rad=" << cfg.task.return_zero_joint_rad
        << ", motor_pos_rad=" << cfg.task.return_zero_motor_pos_rad
        << ", timeout_s=" << cfg.task.return_to_zero_timeout_s << "\n";
    out << "- motor_protocol: address=0x" << std::hex
        << static_cast<unsigned>(cfg.motor.address)
        << ", sequence=0x" << static_cast<unsigned>(cfg.motor.sequence)
        << ", velocity_command=0x" << cfg.motor.velocity_command
        << ", position_command=0x" << cfg.motor.position_command
        << ", feedback_command=0x" << cfg.motor.feedback_command
        << std::dec
        << ", position_counts_per_rev=" << cfg.motor.position_counts_per_rev
        << "\n\n";

    if (!summary.warnings.empty()) {
        out << "## Warnings\n\n";
        for (const auto& warning : summary.warnings)
            out << "- " << warning << "\n";
    }
}

void update_summary_last_values(RunSummary& summary, const float state[26],
                                const float img_pos[6], double velocity_cmd,
                                double joint_cal, bool feedback_ok,
                                bool vision_ok,
                                const std::string& safety_stop_reason)
{
    summary.last_joint_angle_rad = state[0];
    summary.last_joint_velocity_rad_s = state[1];
    for (int i = 0; i < 6; ++i)
        summary.last_img[i] = img_pos[i];
    summary.last_velocity_command_rad_s = velocity_cmd;
    summary.last_joint_cal_rad_s = joint_cal;
    summary.last_feedback_ok = feedback_ok;
    summary.last_vision_ok = vision_ok;
    if (!safety_stop_reason.empty() && safety_stop_reason != "none") {
        summary.safety_limit_triggered = true;
        summary.last_safety_stop_reason = safety_stop_reason;
    }
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <config.yaml>\n", argv[0]);
        return 1;
    }

    AppConfig cfg;
    try {
        cfg = load_app_config(argv[1]);
    } catch (const std::exception& e) {
        fprintf(stderr, "Config error: %s\n", e.what());
        return 1;
    }

    ControllerMode controller_mode = ControllerMode::Proposed;
    try {
        controller_mode = parse_controller_mode(cfg.experiment.controller_mode);
    } catch (const std::exception& e) {
        fprintf(stderr, "Config error: %s\n", e.what());
        return 1;
    }
    cfg.experiment.controller_mode = controller_mode_name(controller_mode);

    std::signal(SIGINT, sig_handler);
    std::signal(SIGTERM, sig_handler);

    RunSummary summary;
    summary.start_wall = WallClock::now();
    summary.start_steady = Clock::now();
    summary.end_wall = summary.start_wall;
    summary.end_steady = summary.start_steady;
    summary.last_joint_angle_rad = cfg.initial_angle_rad;
    summary.last_joint_velocity_rad_s = 0.0;
    summary.return_zero_mode = cfg.task.return_zero_mode;

    const fs::path project_dir = find_project_dir(argv[1]);
    const fs::path log_root = resolve_under_project(project_dir, cfg.log.root_dir);
    try {
        summary.log_dir = make_run_log_dir(log_root, summary.start_wall);
    } catch (const std::exception& e) {
        fprintf(stderr, "Log directory error: %s\n", e.what());
        return 1;
    }

    summary.data_file_path = summary.log_dir / "dataFile.txt";
    summary.config_copy_path = summary.log_dir / "run_config.yaml";
    summary.summary_path = summary.log_dir / "run_summary.md";
    summary.raw_video_path = summary.log_dir / cfg.video.raw_filename;
    summary.annotated_video_path = summary.log_dir / cfg.video.annotated_filename;

    std::error_code copy_ec;
    fs::copy_file(weakly_canonical_or_absolute(argv[1]), summary.config_copy_path,
                  fs::copy_options::overwrite_existing, copy_ec);
    summary.config_copy_ok = !copy_ec;
    if (copy_ec) {
        summary.config_copy_error = copy_ec.message();
        warn(summary, "cannot copy config to " +
                      summary.config_copy_path.string() + ": " +
                      copy_ec.message());
    }

    std::string frame_dir_error;
    const fs::path frame_dir = summary.log_dir / "frames";
    if (!create_directory_checked(frame_dir, frame_dir_error)) {
        warn(summary, "cannot create frame directory " + frame_dir.string() +
                      ": " + frame_dir_error);
    }
    cfg.vision.save_path = frame_dir.string();
    if (!cfg.vision.save_path.empty()) {
        const char last = cfg.vision.save_path[cfg.vision.save_path.size() - 1];
        if (last != '/' && last != '\\')
            cfg.vision.save_path += '/';
    }

    std::ofstream dataFile(summary.data_file_path.string());
    if (!dataFile.is_open()) {
        fprintf(stderr, "Cannot open %s\n", summary.data_file_path.string().c_str());
        return 1;
    }
    write_data_header(dataFile);

    printf("Run log directory: %s\n", summary.log_dir.string().c_str());

    SerialPort port(cfg.serial_port, cfg.baud_rate);
    if (!port.open()) {
        warn(summary, "cannot open serial port " + cfg.serial_port);
        summary.exit_reason = ExitReason::Other;
        summary.end_wall = WallClock::now();
        summary.end_steady = Clock::now();
        dataFile.flush();
        VideoRecorder disabled_raw;
        VideoRecorder disabled_annotated;
        disabled_raw.path = summary.raw_video_path;
        disabled_annotated.path = summary.annotated_video_path;
        write_summary_file(summary, cfg, disabled_raw, disabled_annotated);
        return 1;
    }
    port.flush_input();
    MotorClient motor(port, cfg.motor);

    auto cleanup = [&](ExitReason reason, VideoRecorder& raw_video,
                       VideoRecorder& annotated_video) {
        summary.exit_reason = reason;
        printf("Shutting down: %s\n", exit_reason_name(reason).c_str());

        summary.first_zero_velocity_sent = motor.send_velocity_rad_s(0.0);
        if (!summary.first_zero_velocity_sent)
            warn(summary, "failed to send first zero velocity command");

        summary.return_zero_enabled = cfg.task.return_to_zero_on_exit;
        if (cfg.task.return_to_zero_on_exit) {
            bool target_ok = false;
            if (cfg.task.return_zero_mode == "joint_rad") {
                target_ok = joint_position_to_single_turn_count(
                    cfg.encoder_zero_offset_deg,
                    cfg.task.return_zero_joint_rad,
                    cfg.motor.position_counts_per_rev,
                    summary.return_zero_target_count,
                    summary.return_zero_target_single_turn_deg);
            } else if (cfg.task.return_zero_mode == "motor_pos_rad") {
                target_ok = motor_position_rad_to_single_turn_count(
                    cfg.task.return_zero_motor_pos_rad,
                    cfg.motor.position_counts_per_rev,
                    summary.return_zero_target_count,
                    summary.return_zero_target_single_turn_deg);
            } else {
                warn(summary, "unknown return_zero_mode '" +
                              cfg.task.return_zero_mode +
                              "', expected motor_pos_rad or joint_rad");
            }

            if (!target_ok) {
                warn(summary, "cannot compute return-zero target count");
            } else {
                summary.return_zero_command_sent =
                    motor.send_position_counts(summary.return_zero_target_count);
                if (!summary.return_zero_command_sent) {
                    warn(summary, "failed to send return-zero position command");
                } else {
                    const double timeout_s =
                        std::max(0.0, cfg.task.return_to_zero_timeout_s);
                    const auto deadline = Clock::now() +
                        std::chrono::milliseconds(
                            static_cast<int>(std::llround(timeout_s * 1000.0)));
                    while (Clock::now() < deadline) {
                        MotorFeedback feedback;
                        if (motor.read_system_status(cfg.encoder_zero_offset_deg,
                                                     feedback, 100, true)) {
                            summary.last_joint_angle_rad =
                                feedback.joint_position_rad;
                            summary.last_joint_velocity_rad_s =
                                feedback.velocity_rad_s;
                            const double pos_err = std::fabs(
                                feedback.joint_position_rad -
                                cfg.task.return_zero_joint_rad);
                            const double motor_pos_err = std::fabs(
                                feedback.absolute_position_rad -
                                cfg.task.return_zero_motor_pos_rad);
                            const bool position_reached =
                                cfg.task.return_zero_mode == "joint_rad" ?
                                    pos_err < 0.02 : motor_pos_err < 0.02;
                            if (position_reached &&
                                std::fabs(feedback.velocity_rad_s) <
                                    std::max(0.03, cfg.task.velocity_tolerance_rad_s)) {
                                summary.return_zero_reached = true;
                                break;
                            }
                        } else {
                            std::this_thread::sleep_for(
                                std::chrono::milliseconds(20));
                        }
                    }
                    if (!summary.return_zero_reached) {
                        warn(summary, "return-zero wait timed out before target was confirmed");
                    }
                }
            }
        }

        summary.final_zero_velocity_sent = motor.send_velocity_rad_s(0.0);
        if (!summary.final_zero_velocity_sent)
            warn(summary, "failed to send final zero velocity command");

        dataFile.flush();
        close_video_writer(raw_video);
        close_video_writer(annotated_video);
        summary.end_wall = WallClock::now();
        summary.end_steady = Clock::now();
        write_summary_file(summary, cfg, raw_video, annotated_video);
    };

    FeatureExtractor extractor(cfg.vision);
    VideoRecorder raw_video;
    raw_video.enabled = cfg.video.save_raw_video;
    raw_video.path = summary.raw_video_path;
    raw_video.status = raw_video.enabled ? "pending" : "disabled";

    VideoRecorder annotated_video;
    annotated_video.enabled = cfg.video.save_annotated_video;
    annotated_video.path = summary.annotated_video_path;
    annotated_video.status = annotated_video.enabled ? "pending" : "disabled";

    if (!extractor.open()) {
        warn(summary, "cannot open camera");
        cleanup(ExitReason::Other, raw_video, annotated_video);
        return 1;
    }

    float img_coord_va[6] = {};
    int rad_va[3] = {};
    for (int i = 0; i < 5; i++) {
        cv::Mat warm_raw;
        cv::Mat warm_annotated;
        extractor.extract(img_coord_va, rad_va, &warm_raw, &warm_annotated);
    }

    std::array<float, 6> previous_img = {};
    for (int i = 0; i < 6; ++i)
        previous_img[i] = img_coord_va[i];
    double previous_img_time_s = std::numeric_limits<double>::quiet_NaN();
    bool have_previous_img = false;

    const float joint_va = cfg.initial_angle_rad;
    const float joint_vel_va = 0.0f;

    float state[26] = {};
    state[0] = joint_va;
    state[1] = joint_vel_va;
    state[2] = img_coord_va[0];
    state[3] = img_coord_va[1];
    state[4] = img_coord_va[2];
    state[5] = img_coord_va[3];
    state[6] = img_coord_va[4];
    state[7] = img_coord_va[5];
    state[8] = 400.0f;
    state[9] = 420.0f;
    state[10] = 400.0f;
    state[11] = 300.0f;
    state[12] = 0.0001f;
    state[13] = 0.0001f;
    state[14] = 0.0001f;
    state[15] = 0.0001f;
    state[16] = 0.0001f;
    state[17] = joint_va;
    state[18] = joint_va;
    state[19] = 0.0f;
    state[20] = 0.0f;
    state[21] = joint_va;

    for (int i = 0; i < 6; ++i)
        summary.last_img[i] = state[2 + i];

    int frame_counter = 0;
    int feedback_fail_streak = 0;
    int vision_fail_streak = 0;
    int stable_frames = 0;
    bool have_feedback = false;
    bool startup_angle_out_of_range = false;
    ExitReason exit_reason = ExitReason::Other;

    while (g_running) {
        const auto loop_wall = WallClock::now();
        const auto loop_steady = Clock::now();
        const double elapsed_s =
            std::chrono::duration<double>(loop_steady - summary.start_steady).count();
        if (cfg.task.max_runtime_s > 0.0 && elapsed_s >= cfg.task.max_runtime_s) {
            exit_reason = ExitReason::MaxRuntime;
            break;
        }
        if (cfg.task.max_control_cycles > 0 &&
            frame_counter >= cfg.task.max_control_cycles) {
            exit_reason = ExitReason::MaxControlCycles;
            break;
        }

        float angle_rad = state[0];
        float vel_rad_s = state[1];
        bool feedback_ok_for_log = have_feedback && feedback_fail_streak == 0;
        ++frame_counter;

        if (frame_counter % kFeedbackPollEvery == 1) {
            MotorFeedback feedback;
            const bool feedback_ok =
                read_configured_feedback(motor, cfg.motor,
                                         cfg.encoder_zero_offset_deg, feedback,
                                         kFeedbackTimeoutMs, true);
            const MotorReadStats stats = motor.last_read_stats();
            feedback_ok_for_log = feedback_ok;
            if (feedback_ok) {
                angle_rad = static_cast<float>(feedback.joint_position_rad);
                vel_rad_s = static_cast<float>(feedback.velocity_rad_s);
                feedback_fail_streak = 0;
                if (!have_feedback) {
                    if (cfg.motor.feedback_command == 0x0B) {
                        printf("Motor feedback locked via 0x0B system status: angle=%.4f rad vel=%.4f rad/s\n",
                               angle_rad, vel_rad_s);
                    } else {
                        printf("Motor feedback locked via 0x%02X: angle=%.4f rad vel=%.4f rad/s\n",
                               static_cast<unsigned>(cfg.motor.feedback_command & 0xFF),
                               angle_rad, vel_rad_s);
                    }
                    const float initial_delta =
                        std::fabs(angle_rad - cfg.initial_angle_rad);
                    if (initial_delta > kInitialAngleWarnRad) {
                        fprintf(stderr,
                                "Motor zero warning: first feedback angle %.4f rad differs from initial_angle_rad %.4f rad by %.4f rad. Check encoder_zero_offset_deg.\n",
                                angle_rad, cfg.initial_angle_rad, initial_delta);
                    }
                    if (angle_rad < cfg.task.min_safe_angle_rad ||
                        angle_rad > cfg.task.max_safe_angle_rad) {
                        startup_angle_out_of_range = true;
                        motor.send_velocity_rad_s(0.0);
                        fprintf(stderr,
                                "Motor safety stop: first feedback angle %.4f rad is outside safe range [%.4f, %.4f] rad; commanded zero velocity.\n",
                                angle_rad, cfg.task.min_safe_angle_rad,
                                cfg.task.max_safe_angle_rad);
                    }
                }
                have_feedback = true;
            } else {
                ++feedback_fail_streak;
                if (feedback_fail_streak == 1 ||
                    feedback_fail_streak % kFeedbackWarnEvery == 0) {
                    fprintf(stderr,
                            "Motor feedback unavailable (%d consecutive polls); keeping last state angle=%.4f rad vel=%.4f rad/s skipped=%d crc_bad=%d addr_bad=%d last_skip=0x%02X len=%u\n",
                            feedback_fail_streak, angle_rad, vel_rad_s,
                            stats.skipped_frames, stats.crc_errors,
                            stats.address_mismatches,
                            static_cast<unsigned>(stats.last_skipped_command),
                            static_cast<unsigned>(stats.last_skipped_payload_len));
                }
            }
        }

        feedback_ok_for_log = have_feedback && feedback_fail_streak == 0;

        state[0] = angle_rad;
        state[1] = vel_rad_s;

        float log_img_pos[6] = {
            state[2], state[3], state[4], state[5], state[6], state[7]
        };

        if (cfg.task.max_feedback_failures > 0 &&
            feedback_fail_streak >= cfg.task.max_feedback_failures) {
            const bool stop_ok = motor.send_velocity_rad_s(0.0);
            state[22] = 0.0f;
            if (!stop_ok)
                warn(summary, "failed to send zero velocity after feedback failures");
            const std::string safety_reason = "feedback_failures";
            write_data_row(dataFile, frame_counter, elapsed_s,
                           unix_time_seconds(loop_wall), state, log_img_pos,
                           0.0, 0.0, false, false, safety_reason);
            update_summary_last_values(summary, state, log_img_pos, 0.0, 0.0,
                                       false, false, safety_reason);
            exit_reason = ExitReason::FeedbackFailures;
            break;
        }

        float joint_state[2] = { state[0], state[1] };
        float para_slow[9];
        for (int i = 0; i < 9; i++) {
            para_slow[i] = state[8 + i];
        }
        float obs[4] = { state[17], state[18], state[19], state[20] };
        float q_c = state[21];
        float ydif[6] = {};

        float img_pos[6] = {};
        int rad_new[3] = {};
        cv::Mat raw_frame;
        cv::Mat annotated_frame;
        const bool vision_ok =
            extractor.extract(img_pos, rad_new, &raw_frame, &annotated_frame);
        write_video_frame(raw_video, raw_frame, cfg.vision.fps,
                          cfg.video.codec, summary);

        if (!vision_ok) {
            ++vision_fail_streak;
            const bool stop_ok = motor.send_velocity_rad_s(0.0);
            state[22] = 0.0f;
            if (!stop_ok)
                warn(summary, "failed to send zero velocity after vision failure");
            const std::string safety_reason = "vision_extract_failed";
            write_video_frame(annotated_video, annotated_frame, cfg.vision.fps,
                              cfg.video.codec, summary);
            fprintf(stderr, "Feature extraction failed, commanded zero velocity%s\n",
                    stop_ok ? "" : " (send failed)");
            write_data_row(dataFile, frame_counter, elapsed_s,
                           unix_time_seconds(loop_wall), state, log_img_pos,
                           0.0, 0.0, feedback_ok_for_log, false,
                           safety_reason);
            update_summary_last_values(summary, state, log_img_pos, 0.0, 0.0,
                                       feedback_ok_for_log, false,
                                       safety_reason);
            if (cfg.task.max_vision_failures > 0 &&
                vision_fail_streak >= cfg.task.max_vision_failures) {
                exit_reason = ExitReason::VisionFailures;
                break;
            }
            continue;
        }
        vision_fail_streak = 0;
        for (int i = 0; i < 6; ++i)
            log_img_pos[i] = img_pos[i];
        if (controller_mode_uses_baseline_pd(controller_mode) &&
            have_previous_img &&
            std::isfinite(previous_img_time_s) &&
            elapsed_s > previous_img_time_s) {
            const double dt = elapsed_s - previous_img_time_s;
            for (int i = 0; i < 6; ++i)
                ydif[i] =
                    static_cast<float>((img_pos[i] - previous_img[i]) / dt);
        }
        for (int i = 0; i < 6; ++i)
            previous_img[i] = img_pos[i];
        previous_img_time_s = elapsed_s;
        have_previous_img = true;

        printf("circles: (%.1f,%.1f,r=%d) (%.1f,%.1f,r=%d) (%.1f,%.1f,r=%d)\n",
               img_pos[0], img_pos[1], rad_new[0],
               img_pos[2], img_pos[3], rad_new[1],
               img_pos[4], img_pos[5], rad_new[2]);

        float joint_cal = 0.0f;
        float para_update[17] = {};
        run_controller_step(controller_mode, cfg.ctrl, joint_state, img_pos,
                            obs, para_slow, q_c, ydif, &joint_cal,
                            para_update);

        const float velocity_limit = static_cast<float>(
            std::max(0.0, cfg.task.velocity_saturation_rad_s));
        float velocity =
            std::clamp(joint_cal, -velocity_limit, velocity_limit);
        std::string safety_reason = "none";

        if (startup_angle_out_of_range) {
            velocity = 0.0f;
            append_reason(safety_reason, "startup_angle_out_of_range");
            printf("Startup Angle Safety Stop Active\n");
        }

        if ((angle_rad > cfg.task.max_safe_angle_rad && velocity > 0.0f) ||
            (angle_rad < cfg.task.min_safe_angle_rad && velocity < 0.0f)) {
            velocity = 0.0f;
            append_reason(safety_reason, "position_limit");
            printf("Position Limit Triggered\n");
        }

        const bool vision_limit =
            feature_points_near_boundary(img_pos,
                                         static_cast<float>(cfg.vision.width),
                                         static_cast<float>(cfg.vision.height),
                                         static_cast<float>(
                                             cfg.task.vision_boundary_margin_px));
        if (vision_limit) {
            velocity = 0.0f;
            append_reason(safety_reason, "vision_boundary_limit");
            printf("Vision Boundary Limit Triggered\n");
        }

        bool task_completed = false;
        if (cfg.task.enable_completion_check) {
            const double rms = image_rms_error(img_pos, cfg.ctrl.yd);
            const bool close_to_target =
                rms < std::max(0.0, cfg.task.image_error_tolerance_px);
            const bool slow_enough =
                have_feedback && feedback_fail_streak == 0 &&
                std::fabs(static_cast<double>(vel_rad_s)) <
                    std::max(0.0, cfg.task.velocity_tolerance_rad_s);
            if (close_to_target && slow_enough) {
                ++stable_frames;
            } else {
                stable_frames = 0;
            }
            if (stable_frames >= std::max(1, cfg.task.stable_frames_required)) {
                velocity = 0.0f;
                task_completed = true;
            }
        }

        printf("vel_cmd=%.4f rad/s (joint_cal=%.4f)\n", velocity, joint_cal);

        const bool command_ok = motor.send_velocity_rad_s(velocity);
        if (!command_ok)
            warn(summary, "failed to send velocity command");
        printf("angle=%.4f rad  vel=%.4f rad/s\n", angle_rad, vel_rad_s);

        for (int i = 0; i < 6; i++) {
            state[2 + i] = img_pos[i];
        }
        for (int i = 0; i < 9; i++) {
            state[8 + i] = para_update[i];
        }
        state[17] = para_update[9];
        state[18] = para_update[10];
        state[19] = para_update[11];
        state[20] = para_update[12];
        state[21] = para_update[13];
        state[22] = velocity;
        state[23] = para_update[14];
        state[24] = para_update[15];
        state[25] = para_update[16];

        write_video_frame(annotated_video, annotated_frame, cfg.vision.fps,
                          cfg.video.codec, summary);

        write_data_row(dataFile, frame_counter, elapsed_s,
                       unix_time_seconds(loop_wall), state, img_pos,
                       velocity, joint_cal, feedback_ok_for_log, true,
                       safety_reason);
        update_summary_last_values(summary, state, img_pos, velocity, joint_cal,
                                   feedback_ok_for_log, true, safety_reason);

        if (task_completed) {
            exit_reason = ExitReason::TaskCompleted;
            break;
        }
    }

    summary.total_frames = frame_counter;
    if (!g_running && exit_reason == ExitReason::Other)
        exit_reason = ExitReason::CtrlC;
    cleanup(exit_reason, raw_video, annotated_video);
    return 0;
}
