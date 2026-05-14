#include "camera_utils.hpp"

#include <cstdio>
#include <string>

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
