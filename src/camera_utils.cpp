#include "camera_utils.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <string>

#ifdef __linux__
#include <cerrno>
#include <cstdint>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace {

std::string backend_name(const cv::VideoCapture& cap)
{
#if CV_VERSION_MAJOR >= 4
    return cap.getBackendName();
#else
    (void)cap;
    return "unknown";
#endif
}

void fourcc_to_string(int fourcc, char out[5])
{
    out[0] = static_cast<char>(fourcc & 0xFF);
    out[1] = static_cast<char>((fourcc >> 8) & 0xFF);
    out[2] = static_cast<char>((fourcc >> 16) & 0xFF);
    out[3] = static_cast<char>((fourcc >> 24) & 0xFF);
    out[4] = '\0';
    for (int i = 0; i < 4; ++i) {
        if (out[i] < 32 || out[i] > 126)
            out[i] = '?';
    }
}

#ifdef __linux__

std::string canonical_control_key(const std::string& input)
{
    std::string out;
    out.reserve(input.size());
    for (char ch : input) {
        if ((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
            (ch >= '0' && ch <= '9')) {
            out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
        }
    }
    return out;
}

bool get_control_value(int fd, uint32_t id, int& value)
{
    v4l2_control ctrl = {};
    ctrl.id = id;
    if (::ioctl(fd, VIDIOC_G_CTRL, &ctrl) != 0)
        return false;
    value = ctrl.value;
    return true;
}

bool set_control_value(int fd, uint32_t id, int value)
{
    v4l2_control ctrl = {};
    ctrl.id = id;
    ctrl.value = value;
    return ::ioctl(fd, VIDIOC_S_CTRL, &ctrl) == 0;
}

enum class ControlApplyResult {
    NotFound,
    Applied,
    Failed,
};

template <typename Fn>
void for_each_query_control(int fd, const Fn& fn)
{
    v4l2_queryctrl query = {};
    query.id = V4L2_CTRL_FLAG_NEXT_CTRL;
    while (::ioctl(fd, VIDIOC_QUERYCTRL, &query) == 0) {
        const uint32_t last_id = query.id;
        if (fn(query))
            return;
        query = {};
        query.id = last_id | V4L2_CTRL_FLAG_NEXT_CTRL;
    }

    for (uint32_t id = V4L2_CID_BASE; id < V4L2_CID_LASTP1; ++id) {
        query = {};
        query.id = id;
        if (::ioctl(fd, VIDIOC_QUERYCTRL, &query) == 0 && fn(query))
            return;
    }
    for (uint32_t id = V4L2_CID_CAMERA_CLASS_BASE;
         id < V4L2_CID_CAMERA_CLASS_BASE + 64; ++id) {
        query = {};
        query.id = id;
        if (::ioctl(fd, VIDIOC_QUERYCTRL, &query) == 0 && fn(query))
            return;
    }
}

ControlApplyResult try_apply_control(int fd, const std::string& device,
                                     const std::string& key, int requested_value,
                                     const char* owner, bool verbose)
{
    ControlApplyResult result = ControlApplyResult::NotFound;
    for_each_query_control(fd, [&](const v4l2_queryctrl& query) {
        const std::string query_name =
            reinterpret_cast<const char*>(query.name);
        if (canonical_control_key(query_name) != key)
            return false;

        if (query.flags & V4L2_CTRL_FLAG_DISABLED) {
            fprintf(stderr, "%s: camera control %s is disabled on %s\n",
                    owner, query_name.c_str(), device.c_str());
            result = ControlApplyResult::Failed;
            return true;
        }

        const int step = std::max(1, query.step);
        int value = requested_value;
        value = std::clamp(value, query.minimum, query.maximum);
        if (query.type == V4L2_CTRL_TYPE_BOOLEAN)
            value = value != 0 ? 1 : 0;
        if (query.type != V4L2_CTRL_TYPE_BOOLEAN)
            value = query.minimum + ((value - query.minimum) / step) * step;

        if (!set_control_value(fd, query.id, value)) {
            fprintf(stderr,
                    "%s: failed to set camera control %s=%d on %s: %s\n",
                    owner, query_name.c_str(), value, device.c_str(),
                    std::strerror(errno));
            result = ControlApplyResult::Failed;
            return true;
        }

        int current = value;
        if (!get_control_value(fd, query.id, current))
            current = value;
        if (verbose) {
            fprintf(stderr,
                    "%s: camera control %s=%d applied on %s%s\n",
                    owner, query_name.c_str(), current, device.c_str(),
                    current != requested_value ? " (adjusted)" : "");
        }
        result = ControlApplyResult::Applied;
        return true;
    });
    return result;
}

#endif

} // namespace

bool open_configured_camera(cv::VideoCapture& cap,
                            const FeatureExtractor::Config& cfg,
                            const char* owner,
                            bool verbose)
{
#ifdef __linux__
    cap.open(cfg.camera_index, cv::CAP_V4L2);
    if (!cap.isOpened())
        cap.open(cfg.camera_index);
#else
    cap.open(cfg.camera_index);
#endif

    if (!cap.isOpened()) {
        fprintf(stderr, "%s: cannot open camera %d\n", owner, cfg.camera_index);
        return false;
    }

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, cfg.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.height);
    cap.set(cv::CAP_PROP_FPS, cfg.fps);
    apply_camera_controls(cfg, owner, verbose);

    if (verbose) {
        char fourcc[5] = {};
        const std::string backend = backend_name(cap);
        fourcc_to_string(static_cast<int>(cap.get(cv::CAP_PROP_FOURCC)), fourcc);
        fprintf(stderr,
                "%s: camera %d backend=%s requested=%dx%d@%dfps MJPG "
                "actual=%.0fx%.0f@%.2ffps fourcc=%s\n",
                owner, cfg.camera_index, backend.c_str(),
                cfg.width, cfg.height, cfg.fps,
                cap.get(cv::CAP_PROP_FRAME_WIDTH),
                cap.get(cv::CAP_PROP_FRAME_HEIGHT),
                cap.get(cv::CAP_PROP_FPS),
                fourcc);
    }

    return true;
}

bool apply_camera_controls(const FeatureExtractor::Config& cfg,
                           const char* owner,
                           bool verbose)
{
    if (cfg.camera_controls.empty())
        return true;

#ifdef __linux__
    const std::string device = "/dev/video" + std::to_string(cfg.camera_index);
    const int fd = ::open(device.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "%s: cannot open %s for camera controls: %s\n",
                owner, device.c_str(), std::strerror(errno));
        return false;
    }

    for (const auto& entry : cfg.camera_controls) {
        const std::string key = canonical_control_key(entry.first);
        if (key.empty())
            continue;
        const ControlApplyResult result =
            try_apply_control(fd, device, key, entry.second, owner, verbose);
        if (result == ControlApplyResult::NotFound) {
            fprintf(stderr,
                    "%s: camera control %s not found on %s\n",
                    owner, entry.first.c_str(), device.c_str());
        }
    }

    ::close(fd);
    return true;
#else
    (void)cfg;
    if (verbose) {
        fprintf(stderr,
                "%s: camera_controls configured but automatic V4L2 camera controls are only supported on Linux\n",
                owner);
    }
    return false;
#endif
}
