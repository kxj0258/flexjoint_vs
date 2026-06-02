#include "app_config.hpp"
#include "camera_utils.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <ctime>
#include <exception>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <yaml-cpp/yaml.h>

namespace {

struct Options {
    std::string config_path;
    std::string intrinsics_path;
    std::string output_path = "data/camera_calibration.yaml";
    bool        output_path_from_cli = false;
    int         board_cols = 8;
    int         board_rows = 11;
    int         target_samples = 20;
    double      square_size_m = 0.015;
    bool        extrinsic_only = false;
};

struct CapturedSample {
    std::string image_file;
    std::string annotated_file;
    std::vector<cv::Point2f> corners;
};

void print_usage(const char* argv0)
{
    printf("Usage: %s <config.yaml> [--cols N] [--rows N] [--square M]\n", argv0);
    printf("       [--samples N] [--output PATH] [--intrinsics PATH] [--extrinsic-only]\n");
    printf("Captured images are saved by default under data/camera_calibration_samples/YYYYmmdd_HHMMSS/\n");
    printf("Keys: SPACE capture, k calibrate intrinsics, e solve extrinsics, s save, q/ESC quit\n");
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
        if (arg == "--cols" && i + 1 < argc) {
            opts.board_cols = std::stoi(argv[++i]);
        } else if (arg == "--rows" && i + 1 < argc) {
            opts.board_rows = std::stoi(argv[++i]);
        } else if (arg == "--square" && i + 1 < argc) {
            opts.square_size_m = std::stod(argv[++i]);
        } else if (arg == "--samples" && i + 1 < argc) {
            opts.target_samples = std::stoi(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            opts.output_path = argv[++i];
            opts.output_path_from_cli = true;
        } else if (arg == "--intrinsics" && i + 1 < argc) {
            opts.intrinsics_path = argv[++i];
        } else if (arg == "--extrinsic-only") {
            opts.extrinsic_only = true;
        } else if (arg == "-h" || arg == "--help") {
            return false;
        } else {
            fprintf(stderr, "Unknown argument: %s\n", arg.c_str());
            return false;
        }
    }
    return opts.board_cols > 1 && opts.board_rows > 1 && opts.square_size_m > 0.0;
}

std::vector<cv::Point3f> make_board_points(const Options& opts)
{
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<size_t>(opts.board_cols * opts.board_rows));
    for (int r = 0; r < opts.board_rows; ++r) {
        for (int c = 0; c < opts.board_cols; ++c) {
            points.emplace_back(static_cast<float>(c * opts.square_size_m),
                                static_cast<float>(r * opts.square_size_m),
                                0.0f);
        }
    }
    return points;
}

cv::Mat intrinsics_from_config(const AppConfig& app)
{
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0, 0) = app.ctrl.fx;
    camera_matrix.at<double>(1, 1) = app.ctrl.fy;
    camera_matrix.at<double>(0, 2) = app.ctrl.cx;
    camera_matrix.at<double>(1, 2) = app.ctrl.cy;
    return camera_matrix;
}

bool load_intrinsics_file(const std::string& path, cv::Mat& camera_matrix,
                          cv::Mat& dist_coeffs)
{
    YAML::Node y = YAML::LoadFile(path);

    if (y["camera_matrix"]) {
        const auto values = y["camera_matrix"].as<std::vector<double>>();
        if (values.size() != 9)
            return false;
        camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                camera_matrix.at<double>(r, c) = values[static_cast<size_t>(r * 3 + c)];
    } else if (y["camera_intrinsics"]) {
        const auto values = y["camera_intrinsics"].as<std::vector<double>>();
        if (values.size() != 4)
            return false;
        camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix.at<double>(0, 0) = values[0];
        camera_matrix.at<double>(1, 1) = values[1];
        camera_matrix.at<double>(0, 2) = values[2];
        camera_matrix.at<double>(1, 2) = values[3];
    } else if (y["robot"] && y["robot"]["camera_intrinsics"]) {
        const auto values = y["robot"]["camera_intrinsics"].as<std::vector<double>>();
        if (values.size() != 4)
            return false;
        camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix.at<double>(0, 0) = values[0];
        camera_matrix.at<double>(1, 1) = values[1];
        camera_matrix.at<double>(0, 2) = values[2];
        camera_matrix.at<double>(1, 2) = values[3];
    } else {
        return false;
    }

    if (y["distortion_coeffs"]) {
        const auto values = y["distortion_coeffs"].as<std::vector<double>>();
        dist_coeffs = cv::Mat(values).clone().reshape(1, 1);
    } else {
        dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    }
    return true;
}

bool calibrate_intrinsics(const std::vector<std::vector<cv::Point3f>>& object_samples,
                          const std::vector<std::vector<cv::Point2f>>& image_samples,
                          const cv::Size& image_size,
                          cv::Mat& camera_matrix, cv::Mat& dist_coeffs,
                          double& rms_error)
{
    if (object_samples.size() < 3)
        return false;

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs = cv::Mat::zeros(1, 8, CV_64F);
    rms_error = cv::calibrateCamera(object_samples, image_samples, image_size,
                                    camera_matrix, dist_coeffs, rvecs, tvecs);
    return true;
}

bool solve_extrinsics(const std::vector<cv::Point3f>& object_points,
                      const std::vector<cv::Point2f>& image_points,
                      const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
                      cv::Mat& rvec, cv::Mat& tvec)
{
    if (camera_matrix.empty() || image_points.size() != object_points.size())
        return false;
    return cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs,
                        rvec, tvec);
}

cv::Point text_point(const cv::Point2f& p, const cv::Size& image_size)
{
    const int margin = 8;
    const int x = std::max(margin, std::min(image_size.width - margin,
                                            cvRound(p.x) + margin));
    const int y = std::max(margin, std::min(image_size.height - margin,
                                            cvRound(p.y) - margin));
    return cv::Point(x, y);
}

void draw_label(cv::Mat& frame, const cv::Point& p, const char* text,
                const cv::Scalar& color)
{
    cv::putText(frame, text, p, cv::FONT_HERSHEY_SIMPLEX, 0.58,
                cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    cv::putText(frame, text, p, cv::FONT_HERSHEY_SIMPLEX, 0.58,
                color, 1, cv::LINE_AA);
}

void draw_board_axes(cv::Mat& frame, const cv::Mat& camera_matrix,
                     const cv::Mat& dist_coeffs, const cv::Mat& rvec,
                     const cv::Mat& tvec, double square_size_m)
{
    const float axis_len = static_cast<float>(square_size_m * 3.0);
    std::vector<cv::Point3f> axis_points = {
        cv::Point3f(0.0f, 0.0f, 0.0f),
        cv::Point3f(axis_len, 0.0f, 0.0f),
        cv::Point3f(0.0f, axis_len, 0.0f),
        cv::Point3f(0.0f, 0.0f, axis_len),
    };
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs,
                      image_points);
    if (image_points.size() != axis_points.size())
        return;

    const cv::Point2f origin = image_points[0];
    const cv::Scalar x_color(0, 0, 255);
    const cv::Scalar y_color(0, 200, 0);
    const cv::Scalar z_color(255, 0, 0);

    cv::circle(frame, origin, 5, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
    cv::circle(frame, origin, 5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    cv::arrowedLine(frame, origin, image_points[1], x_color, 3, cv::LINE_AA,
                    0, 0.15);
    cv::arrowedLine(frame, origin, image_points[2], y_color, 3, cv::LINE_AA,
                    0, 0.15);
    cv::arrowedLine(frame, origin, image_points[3], z_color, 3, cv::LINE_AA,
                    0, 0.15);

    const cv::Size size = frame.size();
    draw_label(frame, text_point(origin, size), "O", cv::Scalar(20, 20, 20));
    draw_label(frame, text_point(image_points[1], size), "+X", x_color);
    draw_label(frame, text_point(image_points[2], size), "+Y", y_color);
    draw_label(frame, text_point(image_points[3], size), "+Z", z_color);
}

bool create_directories(const std::string& dir)
{
    if (dir.empty())
        return true;
    std::string current;
    for (char ch : dir) {
        current.push_back(ch);
        if (ch != '/' && ch != '\\')
            continue;
        if (current.size() <= 1)
            continue;
#ifdef _WIN32
        if (_mkdir(current.c_str()) != 0 && errno != EEXIST)
            return false;
#else
        if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
            return false;
#endif
    }
#ifdef _WIN32
    return _mkdir(dir.c_str()) == 0 || errno == EEXIST;
#else
    return mkdir(dir.c_str(), 0755) == 0 || errno == EEXIST;
#endif
}

bool create_parent_directory(const std::string& path)
{
    const size_t slash = path.find_last_of("/\\");
    return slash == std::string::npos || create_directories(path.substr(0, slash));
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

std::string format_time(const char* fmt)
{
    const std::time_t now = std::time(nullptr);
    const std::tm tm_now = local_time(now);
    std::ostringstream oss;
    oss << std::put_time(&tm_now, fmt);
    return oss.str();
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

std::string sample_filename(int index, bool annotated)
{
    std::ostringstream oss;
    oss << "sample_" << std::setw(4) << std::setfill('0') << index;
    if (annotated)
        oss << "_annotated";
    oss << ".png";
    return oss.str();
}

bool save_sample_images(const std::string& sample_dir, int index,
                        const cv::Mat& raw_frame, const cv::Size& board_size,
                        const std::vector<cv::Point2f>& corners,
                        CapturedSample& sample)
{
    sample.image_file = sample_filename(index, false);
    sample.annotated_file = sample_filename(index, true);
    sample.corners = corners;

    const std::string image_path = sample_dir + "/" + sample.image_file;
    const std::string annotated_path = sample_dir + "/" + sample.annotated_file;
    if (!cv::imwrite(image_path, raw_frame))
        return false;

    cv::Mat annotated = raw_frame.clone();
    cv::drawChessboardCorners(annotated, board_size, corners, true);
    return cv::imwrite(annotated_path, annotated);
}

bool write_samples_yaml(const std::string& path, const Options& opts,
                        const cv::Size& image_size,
                        const std::string& calibration_output,
                        const std::string& created_at,
                        const std::vector<CapturedSample>& samples)
{
    if (!create_parent_directory(path))
        return false;

    std::ofstream out(path);
    if (!out.is_open())
        return false;

    out << "board_cols: " << opts.board_cols << "\n";
    out << "board_rows: " << opts.board_rows << "\n";
    out << "square_size_m: " << std::setprecision(10)
        << opts.square_size_m << "\n";
    out << "image_width: " << image_size.width << "\n";
    out << "image_height: " << image_size.height << "\n";
    out << "calibration_output: " << yaml_quote(calibration_output) << "\n";
    out << "created_at: " << yaml_quote(created_at) << "\n";
    if (samples.empty()) {
        out << "samples: []\n";
        return true;
    }

    out << "samples:\n";
    out << std::fixed << std::setprecision(6);
    for (const CapturedSample& sample : samples) {
        out << "  - image: " << yaml_quote(sample.image_file) << "\n";
        out << "    annotated: " << yaml_quote(sample.annotated_file) << "\n";
        out << "    corners: [";
        for (size_t i = 0; i < sample.corners.size(); ++i) {
            if (i != 0)
                out << ", ";
            out << sample.corners[i].x << ", " << sample.corners[i].y;
        }
        out << "]\n";
    }
    return true;
}

bool save_calibration(const std::string& path, const Options& opts,
                      const cv::Size& image_size, const cv::Mat& camera_matrix,
                      const cv::Mat& dist_coeffs, double rms_error,
                      bool have_intrinsics, const cv::Mat& rvec,
                      const cv::Mat& tvec, bool have_extrinsics)
{
    if (!create_parent_directory(path))
        return false;

    std::ofstream out(path);
    if (!out.is_open())
        return false;

    out << "image_width: " << image_size.width << "\n";
    out << "image_height: " << image_size.height << "\n";
    out << "board_cols: " << opts.board_cols << "\n";
    out << "board_rows: " << opts.board_rows << "\n";
    out << "square_size_m: " << opts.square_size_m << "\n";
    out << "reprojection_error: " << rms_error << "\n";

    if (have_intrinsics) {
        out << "camera_intrinsics: ["
            << camera_matrix.at<double>(0, 0) << ", "
            << camera_matrix.at<double>(1, 1) << ", "
            << camera_matrix.at<double>(0, 2) << ", "
            << camera_matrix.at<double>(1, 2) << "]\n";

        out << "camera_matrix: [";
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                if (r != 0 || c != 0)
                    out << ", ";
                out << camera_matrix.at<double>(r, c);
            }
        }
        out << "]\n";

        out << "distortion_coeffs: [";
        cv::Mat dist_row = dist_coeffs.reshape(1, 1);
        for (int i = 0; i < dist_row.cols; ++i) {
            if (i != 0)
                out << ", ";
            out << dist_row.at<double>(0, i);
        }
        out << "]\n";
    }

    if (have_extrinsics) {
        cv::Mat rot;
        cv::Rodrigues(rvec, rot);
        out << "camera_extrinsics: [";
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 4; ++c) {
                if (r != 0 || c != 0)
                    out << ", ";
                out << (c < 3 ? rot.at<double>(r, c) : tvec.at<double>(r, 0));
            }
        }
        out << "]\n";
        out << "extrinsic_frame: \"board/world frame to camera frame\"\n";
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

    AppConfig app;
    try {
        app = load_app_config(opts.config_path);
    } catch (const std::exception& e) {
        fprintf(stderr, "Config error: %s\n", e.what());
        return 1;
    }

    cv::Mat camera_matrix = intrinsics_from_config(app);
    cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    bool have_intrinsics = true;
    if (!opts.intrinsics_path.empty()) {
        try {
            have_intrinsics = load_intrinsics_file(opts.intrinsics_path,
                                                   camera_matrix, dist_coeffs);
        } catch (const std::exception& e) {
            fprintf(stderr, "Cannot load intrinsics file: %s\n", e.what());
            return 1;
        }
        if (!have_intrinsics) {
            fprintf(stderr, "Invalid intrinsics file: %s\n", opts.intrinsics_path.c_str());
            return 1;
        }
    }

    cv::Size image_size(app.vision.width, app.vision.height);
    const std::string project_dir = find_app_project_dir(opts.config_path);
    const std::string output_path = opts.output_path_from_cli
        ? opts.output_path
        : resolve_app_path(project_dir, opts.output_path);
    const std::string sample_dir =
        resolve_app_path(project_dir, "data/camera_calibration_samples/" +
                                      format_time("%Y%m%d_%H%M%S"));
    const std::string samples_yaml_path = sample_dir + "/samples.yaml";
    const std::string created_at = format_time("%Y-%m-%d %H:%M:%S");
    std::vector<CapturedSample> captured_samples;

    if (!create_directories(sample_dir)) {
        fprintf(stderr, "Cannot create sample directory: %s\n", sample_dir.c_str());
        return 1;
    }
    if (!write_samples_yaml(samples_yaml_path, opts, image_size, output_path,
                            created_at, captured_samples)) {
        fprintf(stderr, "Cannot write sample metadata: %s\n",
                samples_yaml_path.c_str());
        return 1;
    }
    printf("Calibration samples will be saved to %s\n", sample_dir.c_str());

    cv::VideoCapture cap;
    if (!open_configured_camera(cap, app.vision, "camera_calibration"))
        return 1;

    const cv::Size board_size(opts.board_cols, opts.board_rows);
    const std::vector<cv::Point3f> board_points = make_board_points(opts);
    std::vector<std::vector<cv::Point3f>> object_samples;
    std::vector<std::vector<cv::Point2f>> image_samples;
    std::vector<cv::Point2f> latest_corners;
    cv::Mat rvec;
    cv::Mat tvec;
    bool have_extrinsics = false;
    double rms_error = 0.0;
    bool calibrated_this_run = false;

    const std::string window = "camera_calibration";
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    printf("Camera calibration started: board=%dx%d square=%.4fm target_samples=%d\n",
           opts.board_cols, opts.board_rows, opts.square_size_m, opts.target_samples);

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            fprintf(stderr, "Empty frame\n");
            break;
        }
        const cv::Mat raw_frame = frame.clone();
        image_size = frame.size();

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        const bool found = cv::findChessboardCorners(
            gray, board_size, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE |
            cv::CALIB_CB_FAST_CHECK);

        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS +
                                              cv::TermCriteria::COUNT, 30, 0.01));
            latest_corners = corners;
            cv::drawChessboardCorners(frame, board_size, corners, found);
            if (have_intrinsics) {
                cv::Mat preview_rvec;
                cv::Mat preview_tvec;
                if (solve_extrinsics(board_points, corners, camera_matrix,
                                     dist_coeffs, preview_rvec, preview_tvec)) {
                    draw_board_axes(frame, camera_matrix, dist_coeffs,
                                    preview_rvec, preview_tvec,
                                    opts.square_size_m);
                }
            }
        }

        char status[220];
        std::snprintf(status, sizeof(status),
                      "found=%s samples=%zu/%d rms=%.4f intr=%s ext=%s",
                      found ? "yes" : "no", image_samples.size(),
                      opts.target_samples, rms_error,
                      have_intrinsics ? "yes" : "no",
                      have_extrinsics ? "yes" : "no");
        cv::putText(frame, status, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX,
                    0.62, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
        cv::putText(frame, status, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX,
                    0.62, cv::Scalar(30, 30, 30), 1, cv::LINE_AA);
        cv::imshow(window, frame);

        if (!opts.extrinsic_only && !calibrated_this_run &&
            static_cast<int>(image_samples.size()) >= opts.target_samples) {
            const bool ok = calibrate_intrinsics(object_samples, image_samples,
                                                 image_size, camera_matrix,
                                                 dist_coeffs, rms_error);
            if (ok) {
                have_intrinsics = true;
                calibrated_this_run = true;
                printf("Intrinsic calibration done. RMS reprojection error = %.6f\n",
                       rms_error);
            }
        }

        const int key = cv::waitKey(10) & 0xFF;
        if (key == 27 || key == 'q')
            break;
        if ((key == ' ' || key == 'c') && found) {
            CapturedSample sample;
            const int sample_index =
                static_cast<int>(captured_samples.size()) + 1;
            if (!save_sample_images(sample_dir, sample_index, raw_frame,
                                    board_size, corners, sample)) {
                fprintf(stderr, "Failed to save sample images in %s\n",
                        sample_dir.c_str());
                continue;
            }
            image_samples.push_back(corners);
            object_samples.push_back(board_points);
            captured_samples.push_back(sample);
            if (!write_samples_yaml(samples_yaml_path, opts, image_size,
                                    output_path, created_at,
                                    captured_samples)) {
                fprintf(stderr, "Failed to update %s\n",
                        samples_yaml_path.c_str());
            }
            calibrated_this_run = false;
            printf("Captured sample %zu: %s\n", image_samples.size(),
                   sample.image_file.c_str());
        }
        if (key == 'k') {
            const bool ok = calibrate_intrinsics(object_samples, image_samples,
                                                 image_size, camera_matrix,
                                                 dist_coeffs, rms_error);
            if (ok) {
                have_intrinsics = true;
                calibrated_this_run = true;
                printf("Intrinsic calibration done. RMS reprojection error = %.6f\n",
                       rms_error);
            } else {
                fprintf(stderr, "Need at least 3 captured samples for intrinsics\n");
            }
        }
        if (key == 'e') {
            have_extrinsics = solve_extrinsics(board_points, latest_corners,
                                               camera_matrix, dist_coeffs,
                                               rvec, tvec);
            if (have_extrinsics)
                printf("Extrinsic solve done. t = [%.6f %.6f %.6f]\n",
                       tvec.at<double>(0, 0), tvec.at<double>(1, 0),
                       tvec.at<double>(2, 0));
            else
                fprintf(stderr, "Cannot solve extrinsics: need visible board and intrinsics\n");
        }
        if (key == 's') {
            if (save_calibration(output_path, opts, image_size,
                                 camera_matrix, dist_coeffs, rms_error,
                                 have_intrinsics, rvec, tvec, have_extrinsics))
                printf("Saved calibration to %s\n", output_path.c_str());
            else
                fprintf(stderr, "Failed to save %s\n", output_path.c_str());
        }
    }

    return 0;
}
