#include "app_config.hpp"
#include "camera_utils.hpp"
#include "feature_detection.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <exception>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#include <direct.h>
#elif defined(__linux__)
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#else
#include <sys/stat.h>
#endif

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {

struct Options {
    std::string config_path;
    std::string tuned_output = "data/vision_tuned.yaml";
    std::string video_output;
    int         print_every = 10;
};

struct Recorder {
    cv::VideoWriter writer;
    std::string     path;
    bool            active = false;
    int             frames = 0;
};

struct CameraControl {
    uint32_t    id = 0;
    std::string name;
    int         min_value = 0;
    int         max_value = 0;
    int         step = 1;
    int         value = 0;
    int         slider = 0;
    int         last_slider = -1;
    bool        inactive = false;
};

void print_usage(const char* argv0)
{
    printf("Usage: %s <config.yaml> [--print-every N] [--save-settings PATH] [--video PATH]\n",
           argv0);
    printf("Keys: q/ESC quit, p pause, s save image, v start/stop MP4 recording, w write tuned settings, r print settings, c print camera controls\n");
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
        if (arg == "--print-every" && i + 1 < argc) {
            opts.print_every = std::stoi(argv[++i]);
        } else if (arg == "--save-settings" && i + 1 < argc) {
            opts.tuned_output = argv[++i];
        } else if (arg == "--video" && i + 1 < argc) {
            opts.video_output = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            return false;
        } else {
            fprintf(stderr, "Unknown argument: %s\n", arg.c_str());
            return false;
        }
    }
    return true;
}

CircleDetectionConfig from_vision_config(const FeatureExtractor::Config& cfg)
{
    CircleDetectionConfig out;
    out.hough_dp       = cfg.hough_dp;
    out.hough_min_dist = cfg.hough_min_dist;
    out.hough_param1   = cfg.hough_param1;
    out.hough_param2   = cfg.hough_param2;
    out.hough_min_rad  = cfg.hough_min_rad;
    out.hough_max_rad  = cfg.hough_max_rad;
    out.blur_kernel    = cfg.blur_kernel;
    out.blur_sigma     = cfg.blur_sigma;
    out.equalize_hist  = cfg.equalize_hist;
    return out;
}

void print_settings(const CircleDetectionConfig& cfg)
{
    printf("settings: dp=%.1f min_dist=%.1f param1=%d param2=%d "
           "min_radius=%d max_radius=%d blur_kernel=%d blur_sigma=%.1f equalize=%d\n",
           cfg.hough_dp, cfg.hough_min_dist, cfg.hough_param1, cfg.hough_param2,
           cfg.hough_min_rad, cfg.hough_max_rad, cfg.blur_kernel, cfg.blur_sigma,
           cfg.equalize_hist ? 1 : 0);
}

int slider_to_control_value(const CameraControl& ctrl)
{
    return ctrl.min_value + ctrl.slider * ctrl.step;
}

std::string short_control_name(const std::string& name)
{
    std::string out;
    out.reserve(name.size());
    for (char ch : name) {
        if (ch == ' ')
            out.push_back('_');
        else
            out.push_back(ch);
    }
    if (out.size() > 32)
        out.resize(32);
    return out;
}

#ifdef __linux__

class V4L2ControlPanel {
public:
    ~V4L2ControlPanel()
    {
        if (fd_ >= 0)
            ::close(fd_);
    }

    bool open(int camera_index)
    {
        device_ = "/dev/video" + std::to_string(camera_index);
        fd_ = ::open(device_.c_str(), O_RDWR | O_NONBLOCK);
        if (fd_ < 0) {
            fprintf(stderr, "V4L2 controls: cannot open %s: %s\n",
                    device_.c_str(), std::strerror(errno));
            return false;
        }
        query_controls();
        return true;
    }

    void create_trackbars(const std::string& window)
    {
        if (controls_.empty()) {
            fprintf(stderr, "V4L2 controls: no writable camera controls found\n");
            return;
        }

        cv::namedWindow(window, cv::WINDOW_NORMAL);
        for (auto& ctrl : controls_) {
            const int slider_max = std::max(1, (ctrl.max_value - ctrl.min_value) / ctrl.step);
            cv::createTrackbar(short_control_name(ctrl.name), window,
                               &ctrl.slider, slider_max);
        }
    }

    void apply_changed()
    {
        for (auto& ctrl : controls_) {
            if (ctrl.slider == ctrl.last_slider)
                continue;

            const int value = slider_to_control_value(ctrl);
            if (set_control(ctrl.id, value)) {
                ctrl.value = value;
                ctrl.last_slider = ctrl.slider;
            } else {
                // Keep the user's slider position; inactive controls often become
                // writable after automatic modes are disabled.
                fprintf(stderr, "V4L2 controls: failed to set %s=%d: %s\n",
                        ctrl.name.c_str(), value, std::strerror(errno));
                ctrl.last_slider = ctrl.slider;
            }
        }
    }

    void refresh_flags()
    {
        for (auto& ctrl : controls_) {
            v4l2_queryctrl query = {};
            query.id = ctrl.id;
            if (::ioctl(fd_, VIDIOC_QUERYCTRL, &query) == 0) {
                const bool was_inactive = ctrl.inactive;
                ctrl.inactive = (query.flags & V4L2_CTRL_FLAG_INACTIVE) != 0;
                if (was_inactive && !ctrl.inactive)
                    ctrl.last_slider = -1;
            }
        }
    }

    void print_controls() const
    {
        printf("camera controls (%s):\n", device_.c_str());
        for (const auto& ctrl : controls_) {
            printf("  %s=%d range=[%d,%d] step=%d%s\n",
                   ctrl.name.c_str(), slider_to_control_value(ctrl),
                   ctrl.min_value, ctrl.max_value, ctrl.step,
                   ctrl.inactive ? " inactive" : "");
        }
    }

    void write_yaml(std::ofstream& out) const
    {
        if (controls_.empty())
            return;
        out << "camera_controls:\n";
        for (const auto& ctrl : controls_)
            out << "  " << short_control_name(ctrl.name) << ": "
                << slider_to_control_value(ctrl) << "\n";
    }

private:
    bool get_control(uint32_t id, int& value) const
    {
        v4l2_control ctrl = {};
        ctrl.id = id;
        if (::ioctl(fd_, VIDIOC_G_CTRL, &ctrl) != 0)
            return false;
        value = ctrl.value;
        return true;
    }

    bool set_control(uint32_t id, int value) const
    {
        v4l2_control ctrl = {};
        ctrl.id = id;
        ctrl.value = value;
        return ::ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == 0;
    }

    void add_control(const v4l2_queryctrl& query)
    {
        if (query.flags & V4L2_CTRL_FLAG_DISABLED)
            return;
        if (query.flags & V4L2_CTRL_FLAG_READ_ONLY)
            return;
        if (query.type != V4L2_CTRL_TYPE_INTEGER &&
            query.type != V4L2_CTRL_TYPE_BOOLEAN &&
            query.type != V4L2_CTRL_TYPE_MENU &&
            query.type != V4L2_CTRL_TYPE_INTEGER_MENU)
            return;

        CameraControl ctrl;
        ctrl.id = query.id;
        ctrl.name = reinterpret_cast<const char*>(query.name);
        ctrl.min_value = query.minimum;
        ctrl.max_value = query.maximum;
        ctrl.step = std::max(1, query.step);
        ctrl.value = query.default_value;
        ctrl.inactive = (query.flags & V4L2_CTRL_FLAG_INACTIVE) != 0;

        int current = 0;
        if (get_control(query.id, current))
            ctrl.value = current;

        ctrl.slider = std::max(0, (ctrl.value - ctrl.min_value) / ctrl.step);
        ctrl.last_slider = ctrl.slider;
        controls_.push_back(ctrl);
    }

    void query_controls()
    {
        controls_.clear();

        v4l2_queryctrl query = {};
        query.id = V4L2_CTRL_FLAG_NEXT_CTRL;
        while (::ioctl(fd_, VIDIOC_QUERYCTRL, &query) == 0) {
            const uint32_t last_id = query.id;
            add_control(query);
            query = {};
            query.id = last_id | V4L2_CTRL_FLAG_NEXT_CTRL;
        }

        if (!controls_.empty())
            return;

        // Fallback for older V4L2 drivers that do not support NEXT_CTRL.
        for (uint32_t id = V4L2_CID_BASE; id < V4L2_CID_LASTP1; ++id) {
            query = {};
            query.id = id;
            if (::ioctl(fd_, VIDIOC_QUERYCTRL, &query) == 0)
                add_control(query);
        }
        for (uint32_t id = V4L2_CID_CAMERA_CLASS_BASE;
             id < V4L2_CID_CAMERA_CLASS_BASE + 64; ++id) {
            query = {};
            query.id = id;
            if (::ioctl(fd_, VIDIOC_QUERYCTRL, &query) == 0)
                add_control(query);
        }
    }

    int fd_ = -1;
    std::string device_;
    std::vector<CameraControl> controls_;
};

#else

class V4L2ControlPanel {
public:
    bool open(int) { return false; }
    void create_trackbars(const std::string&) {}
    void apply_changed() {}
    void refresh_flags() {}
    void print_controls() const
    {
        printf("camera controls: V4L2 control panel is only available on Linux\n");
    }
    void write_yaml(std::ofstream&) const {}
};

#endif

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

std::string make_timestamped_video_path()
{
    std::time_t now = std::time(nullptr);
    std::tm tm_now = {};
#ifdef _WIN32
    localtime_s(&tm_now, &now);
#else
    localtime_r(&now, &tm_now);
#endif

    std::ostringstream oss;
    oss << "data/videos/camera_feature_test_"
        << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << ".mp4";
    return oss.str();
}

std::string normalize_video_path(const std::string& requested)
{
    if (requested.empty())
        return make_timestamped_video_path();
    const size_t slash = requested.find_last_of("/\\");
    const size_t dot = requested.find_last_of('.');
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash))
        return requested + ".mp4";
    return requested;
}

double get_recording_fps(const cv::VideoCapture& cap,
                         const FeatureExtractor::Config& cfg)
{
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps < 1.0 || fps > 300.0)
        fps = cfg.fps > 0 ? cfg.fps : 30.0;
    fps = std::floor(fps);
    return std::max(1.0, fps);
}

bool start_recording(Recorder& recorder, const std::string& requested_path,
                     const cv::Size& size, double fps)
{
    if (recorder.active)
        return true;

    const std::string path = normalize_video_path(requested_path);
    if (!create_parent_directory(path)) {
        fprintf(stderr, "Recording: cannot create output directory for %s\n",
                path.c_str());
        return false;
    }

    const int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    recorder.writer.open(path, fourcc, fps, size, true);
    if (!recorder.writer.isOpened()) {
        fprintf(stderr, "Recording: cannot open MP4 writer for %s\n",
                path.c_str());
        return false;
    }

    recorder.path = path;
    recorder.frames = 0;
    recorder.active = true;
    printf("Recording started: %s %.0fx%.0f @ %.2f fps\n",
           recorder.path.c_str(), static_cast<double>(size.width),
           static_cast<double>(size.height), fps);
    return true;
}

void stop_recording(Recorder& recorder)
{
    if (!recorder.active)
        return;
    recorder.writer.release();
    recorder.active = false;
    printf("Recording stopped: %s (%d frames)\n",
           recorder.path.c_str(), recorder.frames);
}

bool write_tuned_settings(const std::string& path, const CircleDetectionConfig& cfg,
                          const V4L2ControlPanel& controls)
{
    if (!create_parent_directory(path))
        return false;

    std::ofstream out(path);
    if (!out.is_open())
        return false;

    out << "vision:\n";
    out << "  hough_dp: " << cfg.hough_dp << "\n";
    out << "  hough_min_dist: " << cfg.hough_min_dist << "\n";
    out << "  hough_param1: " << cfg.hough_param1 << "\n";
    out << "  hough_param2: " << cfg.hough_param2 << "\n";
    out << "  hough_min_radius: " << cfg.hough_min_rad << "\n";
    out << "  hough_max_radius: " << cfg.hough_max_rad << "\n";
    out << "  blur_kernel: " << cfg.blur_kernel << "\n";
    out << "  blur_sigma: " << cfg.blur_sigma << "\n";
    out << "  equalize_hist: " << (cfg.equalize_hist ? "true" : "false") << "\n";
    controls.write_yaml(out);
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

    cv::VideoCapture cap;
    if (!open_configured_camera(cap, app.vision, "camera_feature_test"))
        return 1;

    V4L2ControlPanel camera_controls;
    const bool has_camera_controls = camera_controls.open(app.vision.camera_index);

    CircleDetectionConfig initial = from_vision_config(app.vision);
    int dp_x10     = static_cast<int>(initial.hough_dp * 10.0);
    int min_dist   = static_cast<int>(initial.hough_min_dist);
    int param1     = initial.hough_param1;
    int param2     = initial.hough_param2;
    int min_radius = initial.hough_min_rad;
    int max_radius = initial.hough_max_rad;
    int blur       = initial.blur_kernel;
    int sigma_x10  = static_cast<int>(initial.blur_sigma * 10.0);
    int equalize   = initial.equalize_hist ? 1 : 0;

    const std::string window = "camera_feature_test";
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    cv::createTrackbar("dp x10", window, &dp_x10, 40);
    cv::createTrackbar("min_dist", window, &min_dist, 300);
    cv::createTrackbar("param1", window, &param1, 300);
    cv::createTrackbar("param2", window, &param2, 200);
    cv::createTrackbar("min_radius", window, &min_radius, 300);
    cv::createTrackbar("max_radius", window, &max_radius, 300);
    cv::createTrackbar("blur", window, &blur, 31);
    cv::createTrackbar("sigma x10", window, &sigma_x10, 100);
    cv::createTrackbar("equalize", window, &equalize, 1);
    if (has_camera_controls)
        camera_controls.create_trackbars("camera_controls");

    printf("Camera feature test started. ");
    print_settings(initial);
    if (has_camera_controls)
        camera_controls.print_controls();

    bool paused = false;
    int frame_index = 0;
    cv::Mat frame;
    int refresh_counter = 0;
    Recorder recorder;

    while (true) {
        if (has_camera_controls) {
            camera_controls.apply_changed();
            if (++refresh_counter % 60 == 0)
                camera_controls.refresh_flags();
        }

        if (!paused) {
            cap >> frame;
            if (frame.empty()) {
                fprintf(stderr, "Empty frame\n");
                break;
            }
            ++frame_index;
        }
        if (frame.empty())
            continue;

        CircleDetectionConfig cfg;
        cfg.hough_dp       = std::max(1, dp_x10) / 10.0;
        cfg.hough_min_dist = std::max(1, min_dist);
        cfg.hough_param1   = std::max(1, param1);
        cfg.hough_param2   = std::max(1, param2);
        cfg.hough_min_rad  = std::max(0, min_radius);
        cfg.hough_max_rad  = std::max(cfg.hough_min_rad + 1, max_radius);
        cfg.blur_kernel    = std::max(1, blur);
        cfg.blur_sigma     = sigma_x10 / 10.0;
        cfg.equalize_hist  = equalize != 0;

        cv::Mat display = frame.clone();
        std::vector<FeatureCircle> circles = detect_feature_circles(frame, cfg);
        std::array<FeatureCircle, 3> selected;
        const bool has_three = select_three_feature_circles(circles, selected);
        draw_feature_overlay(display, circles, has_three ? &selected : nullptr,
                             app.vision.desired);

        char status[160];
        std::snprintf(status, sizeof(status), "circles=%zu selected=%s p:%d/%d r:%d-%d",
                      circles.size(), has_three ? "yes" : "no",
                      cfg.hough_param1, cfg.hough_param2,
                      cfg.hough_min_rad, cfg.hough_max_rad);
        cv::putText(display, status, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX,
                    0.65, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
        cv::putText(display, status, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX,
                    0.65, cv::Scalar(30, 30, 30), 1, cv::LINE_AA);

        if (recorder.active) {
            cv::circle(display, cv::Point(18, 58), 7, cv::Scalar(0, 0, 255), -1);
            cv::putText(display, "REC", cv::Point(32, 64), cv::FONT_HERSHEY_SIMPLEX,
                        0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(display, "REC", cv::Point(32, 64), cv::FONT_HERSHEY_SIMPLEX,
                        0.6, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
            if (!paused) {
                recorder.writer.write(display);
                ++recorder.frames;
            }
        }

        if (has_three && opts.print_every > 0 &&
            frame_index % opts.print_every == 0) {
            float img_pos[6] = {};
            int radius[3] = {};
            feature_circles_to_arrays(selected, img_pos, radius);
            printf("frame=%d circles=%zu "
                   "mid=(%.1f,%.1f,r=%d) max=(%.1f,%.1f,r=%d) min=(%.1f,%.1f,r=%d)\n",
                   frame_index, circles.size(),
                   img_pos[0], img_pos[1], radius[0],
                   img_pos[2], img_pos[3], radius[1],
                   img_pos[4], img_pos[5], radius[2]);
        }

        cv::imshow(window, display);
        const int key = cv::waitKey(paused ? 30 : 1) & 0xFF;
        if (key == 27 || key == 'q')
            break;
        if (key == 'p')
            paused = !paused;
        if (key == 'v') {
            if (recorder.active) {
                stop_recording(recorder);
            } else {
                const double fps = get_recording_fps(cap, app.vision);
                start_recording(recorder, opts.video_output, display.size(), fps);
            }
        }
        if (key == 'r')
            print_settings(cfg);
        if (key == 'c' && has_camera_controls)
            camera_controls.print_controls();
        if (key == 'w') {
            if (write_tuned_settings(opts.tuned_output, cfg, camera_controls))
                printf("Wrote tuned settings to %s\n", opts.tuned_output.c_str());
            else
                fprintf(stderr, "Failed to write %s\n", opts.tuned_output.c_str());
        }
        if (key == 's') {
            create_directories("data/frames");
            const std::string name = "data/frames/camera_test_" +
                                     std::to_string(frame_index) + ".jpg";
            cv::imwrite(name, display);
            printf("Saved %s\n", name.c_str());
        }
    }

    stop_recording(recorder);
    return 0;
}
