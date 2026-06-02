#include "app_config.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#if __has_include(<filesystem>)
#include <filesystem>
#define FLEXJOINT_CALIB_EVAL_HAVE_STD_FILESYSTEM 1
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
#define FLEXJOINT_CALIB_EVAL_HAVE_EXPERIMENTAL_FILESYSTEM 1
#else
#error "C++17 filesystem support is required"
#endif

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace {

#if defined(FLEXJOINT_CALIB_EVAL_HAVE_STD_FILESYSTEM)
namespace fs = std::filesystem;
#else
namespace fs = std::experimental::filesystem;
#endif

struct Options {
    std::string calibration_path;
    std::string sample_dir;
    std::string output_dir;
    bool sample_dir_from_cli = false;
    bool output_dir_from_cli = false;
};

struct Calibration {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
};

struct SampleEntry {
    int index = 0;
    std::string image_file;
    std::string annotated_file;
    std::vector<cv::Point2f> corners;
    bool has_corners = false;
};

struct SampleSet {
    int board_cols = 0;
    int board_rows = 0;
    double square_size_m = 0.0;
    int image_width = 0;
    int image_height = 0;
    std::string calibration_output;
    std::string created_at;
    std::vector<SampleEntry> samples;
};

struct ErrorStats {
    int count = 0;
    double mean = 0.0;
    double max = 0.0;
    double rms = 0.0;
};

struct EvalRecord {
    int index = 0;
    std::string image_file;
    std::string status = "skipped";
    std::string reason;
    std::string overlay_file;
    std::string undistorted_file;
    bool used_saved_corners = false;
    ErrorStats stats;
};

void print_usage(const char* argv0)
{
    printf("Usage: %s <calibration.yaml> [--samples DIR] [--output DIR]\n", argv0);
    printf("If --samples is omitted, the newest data/camera_calibration_samples/YYYYmmdd_HHMMSS directory is used.\n");
}

bool parse_args(int argc, char* argv[], Options& opts)
{
    if (argc < 2)
        return false;
    if (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")
        return false;

    opts.calibration_path = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--samples" && i + 1 < argc) {
            opts.sample_dir = argv[++i];
            opts.sample_dir_from_cli = true;
        } else if (arg == "--output" && i + 1 < argc) {
            opts.output_dir = argv[++i];
            opts.output_dir_from_cli = true;
        } else if (arg == "-h" || arg == "--help") {
            return false;
        } else {
            fprintf(stderr, "Unknown argument: %s\n", arg.c_str());
            return false;
        }
    }
    return true;
}

fs::path absolute_path(const fs::path& path)
{
    if (path.is_absolute())
        return path;
    std::error_code ec;
    const fs::path cwd = fs::current_path(ec);
    if (ec)
        return path;
    return cwd / path;
}

bool first_component_is(const fs::path& path, const char* text)
{
    auto it = path.begin();
    return it != path.end() && it->string() == text;
}

fs::path resolve_cli_path(const fs::path& project_dir, const std::string& value)
{
    fs::path path(value);
    if (path.is_absolute())
        return path;
    if (first_component_is(path, "data"))
        return project_dir / path;

    const fs::path cwd_candidate = absolute_path(path);
    if (fs::exists(cwd_candidate))
        return cwd_candidate;
    if (first_component_is(path, ".") || first_component_is(path, ".."))
        return cwd_candidate;
    return project_dir / path;
}

fs::path resolve_sample_file(const fs::path& sample_dir,
                             const std::string& value)
{
    fs::path path(value);
    if (path.is_absolute())
        return path;
    return sample_dir / path;
}

bool ensure_directory(const fs::path& path, std::string& error)
{
    std::error_code ec;
    fs::create_directories(path, ec);
    if (ec) {
        error = ec.message();
        return false;
    }
    return true;
}

bool is_timestamp_sample_dir(const std::string& name)
{
    if (name.size() < 15 || name[8] != '_')
        return false;
    for (size_t i = 0; i < 15; ++i) {
        if (i == 8)
            continue;
        if (!std::isdigit(static_cast<unsigned char>(name[i])))
            return false;
    }
    return name.size() == 15 || name[15] == '_';
}

bool find_latest_sample_dir(const fs::path& project_dir, fs::path& out,
                            std::string& error)
{
    const fs::path root = project_dir / "data" / "camera_calibration_samples";
    std::error_code ec;
    if (!fs::exists(root, ec) || !fs::is_directory(root, ec)) {
        error = "sample root does not exist: " + root.string();
        return false;
    }

    std::string best_name;
    fs::path best_path;
    try {
        for (const auto& entry : fs::directory_iterator(root)) {
            std::error_code entry_ec;
            if (!fs::is_directory(entry.path(), entry_ec))
                continue;
            const std::string name = entry.path().filename().string();
            if (!is_timestamp_sample_dir(name))
                continue;
            if (best_name.empty() || name > best_name) {
                best_name = name;
                best_path = entry.path();
            }
        }
    } catch (const std::exception& e) {
        error = e.what();
        return false;
    }

    if (best_name.empty()) {
        error = "no timestamped sample directory found under " + root.string();
        return false;
    }
    out = best_path;
    return true;
}

std::string yaml_quote(const std::string& text)
{
    std::ostringstream oss;
    oss << '"';
    for (char ch : text) {
        if (ch == '\\' || ch == '"')
            oss << '\\';
        oss << ch;
    }
    oss << '"';
    return oss.str();
}

std::string csv_field(const std::string& text)
{
    const bool needs_quote = text.find_first_of(",\"\n\r") != std::string::npos;
    if (!needs_quote)
        return text;

    std::ostringstream oss;
    oss << '"';
    for (char ch : text) {
        if (ch == '"')
            oss << '"';
        oss << ch;
    }
    oss << '"';
    return oss.str();
}

std::string numbered_file(const char* prefix, int index)
{
    std::ostringstream oss;
    oss << prefix << "_" << std::setw(4) << std::setfill('0') << index
        << ".png";
    return oss.str();
}

std::vector<double> read_double_sequence(const YAML::Node& node,
                                         const std::string& field)
{
    if (!node || !node.IsSequence())
        throw std::runtime_error("missing or invalid sequence field: " + field);
    return node.as<std::vector<double>>();
}

Calibration load_calibration(const fs::path& path)
{
    YAML::Node y = YAML::LoadFile(path.string());
    Calibration calib;

    if (y["camera_matrix"]) {
        const std::vector<double> values =
            read_double_sequence(y["camera_matrix"], "camera_matrix");
        if (values.size() != 9)
            throw std::runtime_error("camera_matrix must contain 9 numbers");

        calib.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                calib.camera_matrix.at<double>(r, c) =
                    values[static_cast<size_t>(r * 3 + c)];
    } else {
        YAML::Node intrinsics = y["camera_intrinsics"];
        if (!intrinsics && y["robot"])
            intrinsics = y["robot"]["camera_intrinsics"];
        if (!intrinsics)
            throw std::runtime_error("calibration file needs camera_matrix or camera_intrinsics");

        const std::vector<double> values =
            read_double_sequence(intrinsics, "camera_intrinsics");
        if (values.size() != 4)
            throw std::runtime_error("camera_intrinsics must contain [fx, fy, cx, cy]");

        calib.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        calib.camera_matrix.at<double>(0, 0) = values[0];
        calib.camera_matrix.at<double>(1, 1) = values[1];
        calib.camera_matrix.at<double>(0, 2) = values[2];
        calib.camera_matrix.at<double>(1, 2) = values[3];
    }

    if (y["distortion_coeffs"]) {
        const std::vector<double> values =
            read_double_sequence(y["distortion_coeffs"], "distortion_coeffs");
        cv::Mat dist_col(values, true);
        calib.dist_coeffs = dist_col.reshape(1, 1).clone();
    } else {
        calib.dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    }

    return calib;
}

int required_int(const YAML::Node& y, const char* field)
{
    if (!y[field])
        throw std::runtime_error(std::string("samples.yaml missing ") + field);
    return y[field].as<int>();
}

double required_double(const YAML::Node& y, const char* field)
{
    if (!y[field])
        throw std::runtime_error(std::string("samples.yaml missing ") + field);
    return y[field].as<double>();
}

bool read_corners(const YAML::Node& node, size_t expected_points,
                  std::vector<cv::Point2f>& corners)
{
    if (!node || !node.IsSequence())
        return false;

    std::vector<double> values;
    try {
        values = node.as<std::vector<double>>();
    } catch (...) {
        return false;
    }
    if (values.size() != expected_points * 2)
        return false;

    corners.clear();
    corners.reserve(expected_points);
    for (size_t i = 0; i < expected_points; ++i) {
        corners.emplace_back(static_cast<float>(values[2 * i]),
                             static_cast<float>(values[2 * i + 1]));
    }
    return true;
}

SampleSet load_samples_yaml(const fs::path& sample_dir)
{
    const fs::path yaml_path = sample_dir / "samples.yaml";
    YAML::Node y = YAML::LoadFile(yaml_path.string());

    SampleSet set;
    set.board_cols = required_int(y, "board_cols");
    set.board_rows = required_int(y, "board_rows");
    set.square_size_m = required_double(y, "square_size_m");
    if (set.board_cols <= 1 || set.board_rows <= 1 || set.square_size_m <= 0.0)
        throw std::runtime_error("invalid board_cols/board_rows/square_size_m in samples.yaml");

    if (y["image_width"])
        set.image_width = y["image_width"].as<int>();
    if (y["image_height"])
        set.image_height = y["image_height"].as<int>();
    if (y["calibration_output"])
        set.calibration_output = y["calibration_output"].as<std::string>();
    if (y["created_at"])
        set.created_at = y["created_at"].as<std::string>();

    const YAML::Node samples = y["samples"];
    if (!samples || !samples.IsSequence())
        throw std::runtime_error("samples.yaml missing samples list");

    const size_t expected_points =
        static_cast<size_t>(set.board_cols * set.board_rows);
    for (size_t i = 0; i < samples.size(); ++i) {
        const YAML::Node node = samples[i];
        SampleEntry entry;
        entry.index = static_cast<int>(i) + 1;
        if (node["image"])
            entry.image_file = node["image"].as<std::string>();
        if (node["annotated"])
            entry.annotated_file = node["annotated"].as<std::string>();
        entry.has_corners = read_corners(node["corners"], expected_points,
                                         entry.corners);
        set.samples.push_back(entry);
    }

    return set;
}

std::vector<cv::Point3f> make_board_points(const SampleSet& set)
{
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<size_t>(set.board_cols * set.board_rows));
    for (int r = 0; r < set.board_rows; ++r) {
        for (int c = 0; c < set.board_cols; ++c) {
            points.emplace_back(static_cast<float>(c * set.square_size_m),
                                static_cast<float>(r * set.square_size_m),
                                0.0f);
        }
    }
    return points;
}

bool detect_corners(const cv::Mat& image, const cv::Size& board_size,
                    std::vector<cv::Point2f>& corners)
{
    cv::Mat gray;
    if (image.channels() == 1)
        gray = image;
    else
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    const bool found = cv::findChessboardCorners(
        gray, board_size, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE |
        cv::CALIB_CB_FAST_CHECK);
    if (!found)
        return false;

    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS +
                                      cv::TermCriteria::COUNT, 30, 0.01));
    return true;
}

ErrorStats compute_error_stats(const std::vector<cv::Point2f>& observed,
                               const std::vector<cv::Point2f>& projected)
{
    ErrorStats stats;
    if (observed.size() != projected.size() || observed.empty())
        return stats;

    double sum = 0.0;
    double sum_sq = 0.0;
    double max_error = 0.0;
    for (size_t i = 0; i < observed.size(); ++i) {
        const double dx = static_cast<double>(observed[i].x - projected[i].x);
        const double dy = static_cast<double>(observed[i].y - projected[i].y);
        const double err = std::sqrt(dx * dx + dy * dy);
        sum += err;
        sum_sq += err * err;
        max_error = std::max(max_error, err);
    }

    stats.count = static_cast<int>(observed.size());
    stats.mean = sum / observed.size();
    stats.max = max_error;
    stats.rms = std::sqrt(sum_sq / observed.size());
    return stats;
}

void draw_overlay(cv::Mat& overlay,
                  const std::vector<cv::Point2f>& observed,
                  const std::vector<cv::Point2f>& projected)
{
    const cv::Scalar line_color(0, 215, 255);
    const cv::Scalar observed_color(0, 220, 0);
    const cv::Scalar projected_color(255, 0, 255);

    for (size_t i = 0; i < observed.size(); ++i) {
        cv::line(overlay, observed[i], projected[i], line_color, 1,
                 cv::LINE_AA);
        cv::circle(overlay, observed[i], 3, observed_color, -1, cv::LINE_AA);
        cv::circle(overlay, projected[i], 3, projected_color, 1,
                   cv::LINE_AA);
    }
}

bool write_csv(const fs::path& path, const std::vector<EvalRecord>& records)
{
    std::ofstream out(path.string());
    if (!out.is_open())
        return false;

    out << "index,image,status,mean_error_px,max_error_px,rms_error_px,"
           "corner_count,used_saved_corners,overlay,undistorted,reason\n";
    out << std::fixed << std::setprecision(6);
    for (const EvalRecord& record : records) {
        out << record.index << ','
            << csv_field(record.image_file) << ','
            << csv_field(record.status) << ',';
        if (record.status == "ok") {
            out << record.stats.mean << ','
                << record.stats.max << ','
                << record.stats.rms << ','
                << record.stats.count << ','
                << (record.used_saved_corners ? 1 : 0) << ','
                << csv_field(record.overlay_file) << ','
                << csv_field(record.undistorted_file) << ','
                << csv_field(record.reason) << '\n';
        } else {
            out << ",,,0,0,,,";
            out << csv_field(record.reason) << '\n';
        }
    }
    return true;
}

bool write_evaluation_yaml(const fs::path& path,
                           const fs::path& calibration_path,
                           const fs::path& sample_dir,
                           const fs::path& output_dir,
                           const SampleSet& sample_set,
                           const std::vector<EvalRecord>& records,
                           const ErrorStats& overall,
                           int valid_samples)
{
    std::ofstream out(path.string());
    if (!out.is_open())
        return false;

    out << "calibration_file: " << yaml_quote(calibration_path.string()) << "\n";
    out << "sample_dir: " << yaml_quote(sample_dir.string()) << "\n";
    out << "output_dir: " << yaml_quote(output_dir.string()) << "\n";
    out << "board_cols: " << sample_set.board_cols << "\n";
    out << "board_rows: " << sample_set.board_rows << "\n";
    out << "square_size_m: " << std::setprecision(10)
        << sample_set.square_size_m << "\n";
    out << "sample_count: " << records.size() << "\n";
    out << "valid_samples: " << valid_samples << "\n";
    out << "skipped_samples: "
        << static_cast<int>(records.size()) - valid_samples << "\n";
    if (valid_samples > 0) {
        out << std::fixed << std::setprecision(6);
        out << "overall:\n";
        out << "  mean_error_px: " << overall.mean << "\n";
        out << "  max_error_px: " << overall.max << "\n";
        out << "  rms_error_px: " << overall.rms << "\n";
        out << "  corner_count: " << overall.count << "\n";
    } else {
        out << "overall: null\n";
    }

    out << "samples:\n";
    out << std::fixed << std::setprecision(6);
    for (const EvalRecord& record : records) {
        out << "  - index: " << record.index << "\n";
        out << "    image: " << yaml_quote(record.image_file) << "\n";
        out << "    status: " << yaml_quote(record.status) << "\n";
        if (record.status == "ok") {
            out << "    overlay: " << yaml_quote(record.overlay_file) << "\n";
            out << "    undistorted: "
                << yaml_quote(record.undistorted_file) << "\n";
            out << "    used_saved_corners: "
                << (record.used_saved_corners ? "true" : "false") << "\n";
            out << "    corner_count: " << record.stats.count << "\n";
            out << "    mean_error_px: " << record.stats.mean << "\n";
            out << "    max_error_px: " << record.stats.max << "\n";
            out << "    rms_error_px: " << record.stats.rms << "\n";
        } else {
            out << "    reason: " << yaml_quote(record.reason) << "\n";
        }
    }
    return true;
}

} // namespace

int main(int argc, char* argv[])
{
    Options opts;
    if (!parse_args(argc, argv, opts)) {
        print_usage(argv[0]);
        return 1;
    }

    const fs::path calibration_path = absolute_path(opts.calibration_path);
    const fs::path project_dir =
        fs::path(find_app_project_dir(calibration_path.string()));

    fs::path sample_dir;
    if (opts.sample_dir_from_cli) {
        sample_dir = resolve_cli_path(project_dir, opts.sample_dir);
    } else {
        std::string error;
        if (!find_latest_sample_dir(project_dir, sample_dir, error)) {
            fprintf(stderr, "Sample directory error: %s\n", error.c_str());
            return 1;
        }
    }

    fs::path output_dir = opts.output_dir_from_cli
        ? resolve_cli_path(project_dir, opts.output_dir)
        : sample_dir / "eval";

    std::string dir_error;
    if (!ensure_directory(output_dir, dir_error)) {
        fprintf(stderr, "Cannot create output directory %s: %s\n",
                output_dir.string().c_str(), dir_error.c_str());
        return 1;
    }

    Calibration calibration;
    SampleSet sample_set;
    try {
        calibration = load_calibration(calibration_path);
        sample_set = load_samples_yaml(sample_dir);
    } catch (const std::exception& e) {
        fprintf(stderr, "Input error: %s\n", e.what());
        return 1;
    }

    if (sample_set.samples.empty()) {
        fprintf(stderr, "No samples listed in %s\n",
                (sample_dir / "samples.yaml").string().c_str());
        return 1;
    }

    const cv::Size board_size(sample_set.board_cols, sample_set.board_rows);
    const std::vector<cv::Point3f> object_points =
        make_board_points(sample_set);
    const size_t expected_points = object_points.size();

    std::vector<EvalRecord> records;
    records.reserve(sample_set.samples.size());
    double overall_sum = 0.0;
    double overall_sum_sq = 0.0;
    double overall_max = 0.0;
    int overall_points = 0;
    int valid_samples = 0;

    for (const SampleEntry& sample : sample_set.samples) {
        EvalRecord record;
        record.index = sample.index;
        record.image_file = sample.image_file;

        if (sample.image_file.empty()) {
            record.reason = "sample entry is missing image";
            records.push_back(record);
            continue;
        }

        const fs::path image_path = resolve_sample_file(sample_dir,
                                                        sample.image_file);
        cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
        if (image.empty()) {
            record.reason = "cannot load image: " + image_path.string();
            records.push_back(record);
            continue;
        }

        std::vector<cv::Point2f> corners;
        if (sample.has_corners && sample.corners.size() == expected_points) {
            corners = sample.corners;
            record.used_saved_corners = true;
        } else if (!detect_corners(image, board_size, corners)) {
            record.reason = "chessboard detection failed";
            records.push_back(record);
            continue;
        }

        if (corners.size() != expected_points) {
            record.reason = "corner count does not match board size";
            records.push_back(record);
            continue;
        }

        cv::Mat rvec;
        cv::Mat tvec;
        bool pose_ok = false;
        try {
            pose_ok = cv::solvePnP(object_points, corners,
                                   calibration.camera_matrix,
                                   calibration.dist_coeffs, rvec, tvec);
        } catch (const cv::Exception& e) {
            record.reason = std::string("solvePnP failed: ") + e.what();
            records.push_back(record);
            continue;
        }
        if (!pose_ok) {
            record.reason = "solvePnP failed";
            records.push_back(record);
            continue;
        }

        std::vector<cv::Point2f> projected;
        try {
            cv::projectPoints(object_points, rvec, tvec,
                              calibration.camera_matrix,
                              calibration.dist_coeffs, projected);
        } catch (const cv::Exception& e) {
            record.reason = std::string("projectPoints failed: ") + e.what();
            records.push_back(record);
            continue;
        }

        record.stats = compute_error_stats(corners, projected);
        if (record.stats.count <= 0) {
            record.reason = "cannot compute reprojection error";
            records.push_back(record);
            continue;
        }

        record.overlay_file = numbered_file("overlay", sample.index);
        record.undistorted_file = numbered_file("undistorted", sample.index);

        cv::Mat overlay = image.clone();
        draw_overlay(overlay, corners, projected);
        const fs::path overlay_path = output_dir / record.overlay_file;
        if (!cv::imwrite(overlay_path.string(), overlay)) {
            fprintf(stderr, "Warning: failed to write %s\n",
                    overlay_path.string().c_str());
        }

        cv::Mat undistorted;
        cv::undistort(image, undistorted, calibration.camera_matrix,
                      calibration.dist_coeffs);
        const fs::path undistorted_path = output_dir / record.undistorted_file;
        if (!cv::imwrite(undistorted_path.string(), undistorted)) {
            fprintf(stderr, "Warning: failed to write %s\n",
                    undistorted_path.string().c_str());
        }

        record.status = "ok";
        ++valid_samples;
        overall_points += record.stats.count;
        overall_sum += record.stats.mean * record.stats.count;
        overall_sum_sq += record.stats.rms * record.stats.rms *
                          record.stats.count;
        overall_max = std::max(overall_max, record.stats.max);
        records.push_back(record);
    }

    ErrorStats overall;
    overall.count = overall_points;
    if (overall_points > 0) {
        overall.mean = overall_sum / overall_points;
        overall.max = overall_max;
        overall.rms = std::sqrt(overall_sum_sq / overall_points);
    }

    const fs::path yaml_path = output_dir / "evaluation.yaml";
    const fs::path csv_path = output_dir / "reprojection_errors.csv";
    if (!write_evaluation_yaml(yaml_path, calibration_path, sample_dir,
                               output_dir, sample_set, records, overall,
                               valid_samples)) {
        fprintf(stderr, "Failed to write %s\n", yaml_path.string().c_str());
        return 1;
    }
    if (!write_csv(csv_path, records)) {
        fprintf(stderr, "Failed to write %s\n", csv_path.string().c_str());
        return 1;
    }

    if (valid_samples == 0) {
        fprintf(stderr, "No usable calibration samples found; reports were written to %s\n",
                output_dir.string().c_str());
        return 1;
    }

    printf("Evaluated %d/%zu samples\n", valid_samples, sample_set.samples.size());
    printf("Evaluation written to %s\n", output_dir.string().c_str());
    printf("Overall reprojection error: mean=%.6f px max=%.6f px rms=%.6f px\n",
           overall.mean, overall.max, overall.rms);
    return 0;
}
