#include "app_config.hpp"

#include <cstdlib>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#if __has_include(<filesystem>)
#include <filesystem>
#define FLEXJOINT_APP_CONFIG_HAVE_STD_FILESYSTEM 1
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
#define FLEXJOINT_APP_CONFIG_HAVE_EXPERIMENTAL_FILESYSTEM 1
#else
#error "C++17 filesystem support is required"
#endif

#include <yaml-cpp/yaml.h>

namespace {

#if defined(FLEXJOINT_APP_CONFIG_HAVE_STD_FILESYSTEM)
namespace fs = std::filesystem;
#else
namespace fs = std::experimental::filesystem;
#endif

YAML::Node clone_node(const YAML::Node& node)
{
    return YAML::Clone(node);
}

YAML::Node merge_yaml_nodes(const YAML::Node& base, const YAML::Node& overlay)
{
    if (!base || !base.IsMap() || !overlay || !overlay.IsMap())
        return clone_node(overlay ? overlay : base);

    YAML::Node merged = clone_node(base);
    for (const auto& entry : overlay) {
        const std::string key = entry.first.as<std::string>();
        if (key == "extends")
            continue;

        const YAML::Node base_child = merged[key];
        const YAML::Node overlay_child = entry.second;
        if (base_child && base_child.IsMap() && overlay_child.IsMap()) {
            merged[key] = merge_yaml_nodes(base_child, overlay_child);
        } else {
            merged[key] = clone_node(overlay_child);
        }
    }
    return merged;
}

fs::path absolute_config_path(const fs::path& path)
{
    const fs::path abs_path = path.is_absolute() ? path : fs::absolute(path);
#if defined(FLEXJOINT_APP_CONFIG_HAVE_STD_FILESYSTEM)
    return abs_path.lexically_normal();
#else
    return abs_path;
#endif
}

YAML::Node load_resolved_yaml_impl(const fs::path& path,
                                   std::set<std::string>& include_stack)
{
    const fs::path abs_path = absolute_config_path(path);
    const std::string key = abs_path.string();
    if (!include_stack.insert(key).second) {
        throw std::runtime_error("cyclic YAML extends detected at " + key);
    }

    YAML::Node current = YAML::LoadFile(key);
    if (!current || !current.IsMap()) {
        throw std::runtime_error("config file must contain a YAML mapping: " + key);
    }

    YAML::Node resolved = clone_node(current);
    if (const YAML::Node extends = current["extends"]) {
        const fs::path parent_path = abs_path.parent_path() /
            extends.as<std::string>();
        YAML::Node parent = load_resolved_yaml_impl(parent_path, include_stack);
        resolved = merge_yaml_nodes(parent, current);
    }
    resolved.remove("extends");

    include_stack.erase(key);
    return resolved;
}

YAML::Node load_resolved_yaml(const std::string& path)
{
    std::set<std::string> include_stack;
    return load_resolved_yaml_impl(path, include_stack);
}

fs::path find_project_dir(const fs::path& config_path)
{
    fs::path current = absolute_config_path(config_path).parent_path();
    while (!current.empty()) {
        if (fs::exists(current / "CMakeLists.txt") &&
            fs::exists(current / "src") &&
            fs::exists(current / "config")) {
            return current;
        }
        const fs::path parent = current.parent_path();
        if (parent == current)
            break;
        current = parent;
    }

    return absolute_config_path(config_path).parent_path();
}

int scalar_to_int(const YAML::Node& node, int fallback)
{
    if (!node)
        return fallback;
    try {
        const std::string text = node.as<std::string>();
        char* end = nullptr;
        const long value = std::strtol(text.c_str(), &end, 0);
        if (end && *end == '\0')
            return static_cast<int>(value);
    } catch (...) {
    }
    return node.as<int>();
}

double scalar_to_double(const YAML::Node& node, double fallback)
{
    if (!node)
        return fallback;
    return node.as<double>();
}

bool scalar_to_bool(const YAML::Node& node, bool fallback)
{
    if (!node)
        return fallback;
    return node.as<bool>();
}

int validate_feature_count(int feature_count)
{
    if (feature_count != kLegacyFeaturePoints &&
        feature_count != kMaxFeaturePoints) {
        throw std::runtime_error("vision.feature_count must be 3 or 4");
    }
    return feature_count;
}

std::string scalar_to_string(const YAML::Node& node, const std::string& fallback)
{
    if (!node)
        return fallback;
    return node.as<std::string>();
}

} // namespace

AppConfig load_app_config(const std::string& path)
{
    YAML::Node y = load_resolved_yaml(path);
    AppConfig c;

    const auto serial = y["serial"];
    c.serial_port = serial["motor_port"].as<std::string>();
#ifdef _WIN32
    if (serial["windows_port"])
        c.serial_port = serial["windows_port"].as<std::string>();
#else
    if (serial["linux_port"])
        c.serial_port = serial["linux_port"].as<std::string>();
#endif
    c.baud_rate = serial["baud_rate"].as<int>();

    c.initial_angle_rad       = y["robot"]["initial_angle_rad"].as<float>();
    c.encoder_zero_offset_deg = y["robot"]["encoder_zero_offset_deg"].as<float>();

    const auto vis = y["vision"];
    const int feature_count = validate_feature_count(
        scalar_to_int(vis["feature_count"], kLegacyFeaturePoints));
    c.vision.feature_count = feature_count;
    c.ctrl.feature_count = feature_count;

    auto intr = y["robot"]["camera_intrinsics"].as<std::vector<float>>();
    c.ctrl.fx = intr[0];
    c.ctrl.fy = intr[1];
    c.ctrl.cx = intr[2];
    c.ctrl.cy = intr[3];

    auto ex = y["robot"]["camera_extrinsics"].as<std::vector<float>>();
    for (int i = 0; i < 12; i++)
        c.ctrl.cam_ex[i] = ex[i];

    c.ctrl.L = y["robot"]["L"].as<float>();
    auto e1 = y["robot"]["rt_e1"].as<std::vector<float>>();
    auto e2 = y["robot"]["rt_e2"].as<std::vector<float>>();
    for (int i = 0; i < 3; i++) {
        c.ctrl.rt_e1[i] = e1[i];
        c.ctrl.rt_e2[i] = e2[i];
    }
    for (int i = 0; i < 3; ++i) {
        c.ctrl.feature_offsets[0][i] = 0.0f;
        c.ctrl.feature_offsets[1][i] = c.ctrl.rt_e1[i];
        c.ctrl.feature_offsets[2][i] = c.ctrl.rt_e2[i];
        c.ctrl.feature_offsets[3][i] = 0.0f;
    }
    if (const auto offsets = y["robot"]["feature_offsets"]) {
        if (!offsets.IsSequence()) {
            throw std::runtime_error("robot.feature_offsets must be a sequence");
        }
        if (offsets.size() < static_cast<size_t>(feature_count)) {
            std::ostringstream oss;
            oss << "robot.feature_offsets must contain at least "
                << feature_count << " three-number offsets for feature_count="
                << feature_count;
            throw std::runtime_error(oss.str());
        }
        const int offsets_to_copy = static_cast<int>(
            std::min<size_t>(offsets.size(), kMaxFeaturePoints));
        for (int point = 0; point < offsets_to_copy; ++point) {
            auto offset = offsets[point].as<std::vector<float>>();
            if (offset.size() != 3) {
                std::ostringstream oss;
                oss << "robot.feature_offsets[" << point
                    << "] must contain exactly 3 numbers";
                throw std::runtime_error(oss.str());
            }
            for (int axis = 0; axis < 3; ++axis)
                c.ctrl.feature_offsets[point][axis] = offset[axis];
        }
        for (int axis = 0; axis < 3; ++axis) {
            c.ctrl.rt_e1[axis] = c.ctrl.feature_offsets[1][axis];
            c.ctrl.rt_e2[axis] = c.ctrl.feature_offsets[2][axis];
        }
    } else if (feature_count == kMaxFeaturePoints) {
        throw std::runtime_error(
            "vision.feature_count=4 requires robot.feature_offsets with at least 4 offsets");
    }

    const auto ctrl = y["control"];
    c.ctrl.K1        = ctrl["K1"].as<float>();
    c.ctrl.B         = ctrl["B"].as<float>();
    c.ctrl.K2        = ctrl["K2"].as<float>();
    c.ctrl.Gamma     = ctrl["Gamma"].as<float>();
    c.ctrl.K4        = ctrl["K4"].as<float>();
    c.ctrl.eps       = ctrl["eps"].as<float>();
    c.ctrl.Kq        = ctrl["Kq"].as<float>();
    c.ctrl.cmd_c1    = ctrl["cmd_c1"].as<float>();
    c.ctrl.cmd_c2    = ctrl["cmd_c2"].as<float>();
    c.ctrl.K_dtk_I   = ctrl["K_dtk_I"].as<float>();
    c.ctrl.omega_dtk = ctrl["omega_dtk"].as<float>();
    c.ctrl.h         = ctrl["h"].as<float>();
    c.ctrl.Mr        = ctrl["Mr"].as<float>();
    c.ctrl.Jm        = ctrl["Jm"].as<float>();
    c.ctrl.Kp        = ctrl["Kp"].as<float>();
    c.ctrl.Kd        = ctrl["Kd"].as<float>();

    auto ps = ctrl["Ps"].as<std::vector<float>>();
    for (int i = 0; i < 4; i++)
        c.ctrl.Ps[i] = ps[i];

    auto mu = ctrl["mu_rho"].as<std::vector<float>>();
    for (int i = 0; i < 5; i++)
        c.ctrl.mu_rho[i] = mu[i];

    auto yd = vis["desired_coords"].as<std::vector<float>>();
    const int expected_coords = 2 * feature_count;
    if (yd.size() != static_cast<size_t>(expected_coords)) {
        std::ostringstream oss;
        oss << "vision.desired_coords must contain " << expected_coords
            << " numbers when vision.feature_count=" << feature_count
            << ", got " << yd.size();
        throw std::runtime_error(oss.str());
    }
    for (int i = 0; i < expected_coords; i++)
        c.ctrl.yd[i] = yd[i];

    c.vision.camera_index  = y["camera"]["index"].as<int>();
    c.vision.fps           = y["camera"]["fps"].as<int>();
    c.vision.width         = y["camera"]["width"].as<int>();
    c.vision.height        = y["camera"]["height"].as<int>();
    c.vision.hough_param1  = vis["hough_param1"].as<int>();
    c.vision.hough_param2  = vis["hough_param2"].as<int>();
    c.vision.hough_min_rad = vis["hough_min_radius"].as<int>();
    c.vision.hough_max_rad = vis["hough_max_radius"].as<int>();
    c.vision.save_path     = vis["save_path"].as<std::string>();
    if (vis["hough_dp"])
        c.vision.hough_dp = vis["hough_dp"].as<double>();
    if (vis["hough_min_dist"])
        c.vision.hough_min_dist = vis["hough_min_dist"].as<double>();
    if (vis["blur_kernel"])
        c.vision.blur_kernel = vis["blur_kernel"].as<int>();
    if (vis["blur_sigma"])
        c.vision.blur_sigma = vis["blur_sigma"].as<double>();
    if (vis["equalize_hist"])
        c.vision.equalize_hist = vis["equalize_hist"].as<bool>();
    if (const auto controls = y["camera_controls"]) {
        for (const auto& entry : controls) {
            c.vision.camera_controls.emplace_back(entry.first.as<std::string>(),
                                                  scalar_to_int(entry.second, 0));
        }
    }
    for (int i = 0; i < expected_coords; i++)
        c.vision.desired[i] = yd[i];

    const auto proto = y["motor_protocol"];
    if (proto) {
        c.motor.velocity_command = scalar_to_int(proto["velocity_command"],
                                                 c.motor.velocity_command);
        c.motor.position_command = scalar_to_int(proto["position_command"],
                                                 c.motor.position_command);
        c.motor.pid_command = scalar_to_int(proto["pid_command"],
                                            c.motor.pid_command);
        c.motor.feedback_command = scalar_to_int(proto["feedback_command"],
                                                 c.motor.feedback_command);
        c.motor.sequence = static_cast<uint8_t>(
            scalar_to_int(proto["sequence"], c.motor.sequence) & 0xFF);
        c.motor.address = static_cast<uint8_t>(
            scalar_to_int(proto["address"], c.motor.address) & 0xFF);
        if (proto["strict_address"])
            c.motor.strict_address = proto["strict_address"].as<bool>();
        if (proto["consume_command_response"]) {
            c.motor.consume_command_response =
                proto["consume_command_response"].as<bool>();
        }
        if (proto["debug_frames"])
            c.motor.debug_frames = proto["debug_frames"].as<bool>();
        c.motor.position_counts_per_rev =
            scalar_to_double(proto["position_counts_per_rev"],
                             c.motor.position_counts_per_rev);
        c.motor.pid_scale = scalar_to_double(proto["pid_scale"],
                                             c.motor.pid_scale);
    }

    const auto task = y["task"];
    if (task) {
        c.task.enable_completion_check =
            scalar_to_bool(task["enable_completion_check"],
                           c.task.enable_completion_check);
        c.task.image_error_tolerance_px =
            scalar_to_double(task["image_error_tolerance_px"],
                             c.task.image_error_tolerance_px);
        c.task.velocity_tolerance_rad_s =
            scalar_to_double(task["velocity_tolerance_rad_s"],
                             c.task.velocity_tolerance_rad_s);
        c.task.stable_frames_required =
            scalar_to_int(task["stable_frames_required"],
                          c.task.stable_frames_required);
        c.task.max_runtime_s =
            scalar_to_double(task["max_runtime_s"], c.task.max_runtime_s);
        c.task.max_control_cycles =
            scalar_to_int(task["max_control_cycles"],
                          c.task.max_control_cycles);
        c.task.return_to_zero_on_exit =
            scalar_to_bool(task["return_to_zero_on_exit"],
                           c.task.return_to_zero_on_exit);
        c.task.return_zero_mode =
            scalar_to_string(task["return_zero_mode"],
                             c.task.return_zero_mode);
        c.task.return_zero_joint_rad =
            scalar_to_double(task["return_zero_joint_rad"],
                             c.task.return_zero_joint_rad);
        c.task.return_zero_motor_pos_rad =
            scalar_to_double(task["return_zero_motor_pos_rad"],
                             c.task.return_zero_motor_pos_rad);
        c.task.return_to_zero_timeout_s =
            scalar_to_double(task["return_to_zero_timeout_s"],
                             c.task.return_to_zero_timeout_s);
        c.task.max_vision_failures =
            scalar_to_int(task["max_vision_failures"],
                          c.task.max_vision_failures);
        c.task.max_feedback_failures =
            scalar_to_int(task["max_feedback_failures"],
                          c.task.max_feedback_failures);
        c.task.velocity_saturation_rad_s =
            scalar_to_double(task["velocity_saturation_rad_s"],
                             c.task.velocity_saturation_rad_s);
        c.task.min_safe_angle_rad =
            scalar_to_double(task["min_safe_angle_rad"],
                             c.task.min_safe_angle_rad);
        c.task.max_safe_angle_rad =
            scalar_to_double(task["max_safe_angle_rad"],
                             c.task.max_safe_angle_rad);
        c.task.vision_boundary_margin_px =
            scalar_to_double(task["vision_boundary_margin_px"],
                             c.task.vision_boundary_margin_px);
    }

    const auto log = y["log"];
    if (log) {
        c.log.root_dir = scalar_to_string(log["root_dir"], c.log.root_dir);
    }

    const auto video = y["video"];
    if (video) {
        c.video.save_raw_video =
            scalar_to_bool(video["save_raw_video"], c.video.save_raw_video);
        c.video.save_annotated_video =
            scalar_to_bool(video["save_annotated_video"],
                           c.video.save_annotated_video);
        c.video.raw_filename =
            scalar_to_string(video["raw_filename"], c.video.raw_filename);
        c.video.annotated_filename =
            scalar_to_string(video["annotated_filename"],
                             c.video.annotated_filename);
        c.video.codec = scalar_to_string(video["codec"], c.video.codec);
    }

    const auto experiment = y["experiment"];
    if (experiment) {
        c.experiment.controller_mode =
            scalar_to_string(experiment["controller_mode"],
                             c.experiment.controller_mode);
    }

    return c;
}

std::string load_resolved_app_config_text(const std::string& path)
{
    YAML::Emitter out;
    out << load_resolved_yaml(path);
    std::ostringstream text;
    text << out.c_str() << '\n';
    return text.str();
}

std::string find_app_project_dir(const std::string& config_path)
{
    return find_project_dir(config_path).string();
}

std::string resolve_app_path(const std::string& project_dir,
                             const std::string& path)
{
    fs::path p(path);
    if (p.is_absolute())
        return p.string();
    return (fs::path(project_dir) / p).string();
}
