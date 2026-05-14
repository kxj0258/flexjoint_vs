#pragma once
#include <opencv2/videoio.hpp>
#include <string>

// Wraps camera capture and Hough-circle feature extraction.
// Fixes the original bug where VideoCapture was passed by value.
class FeatureExtractor {
public:
    struct Config {
        int   camera_index   = 0;
        int   fps            = 110;
        int   width          = 640;
        int   height         = 480;
        int   hough_param1   = 100;
        int   hough_param2   = 38;
        int   hough_min_rad  = 3;
        int   hough_max_rad  = 80;
        double hough_dp       = 1.0;
        double hough_min_dist = 30.0;
        int   blur_kernel    = 7;
        double blur_sigma     = 2.0;
        bool  equalize_hist   = false;
        float desired[6]     = {264.5f, 96.5f, 298.5f, 166.5f, 174.5f, 144.5f};
        std::string save_path = "data/frames/";
    };

    explicit FeatureExtractor(const Config& cfg);
    ~FeatureExtractor();

    bool open();

    // Capture one frame and extract 3 circles sorted by radius (mid, max, min).
    // img_pos[6]: u1,v1, u2,v2, u3,v3
    // rad_out[3]: radius of mid, max, min circle
    // Returns false if capture or detection fails.
    bool extract(float img_pos[6], int rad_out[3]);

private:
    Config        cfg_;
    cv::VideoCapture cap_;
    int           frame_count_ = 0;
};
