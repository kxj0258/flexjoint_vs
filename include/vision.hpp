#pragma once
#include <opencv2/videoio.hpp>
#include <string>
#include <utility>
#include <vector>

#include "feature_points.hpp"

// Wraps camera capture and Hough-circle feature extraction.
// Fixes the original bug where VideoCapture was passed by value.
class FeatureExtractor {
public:
    struct Config {
        int   feature_count  = kLegacyFeaturePoints;
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
        float desired[kMaxImageCoords] = {
            264.5f, 96.5f, 298.5f, 166.5f, 174.5f, 144.5f, 0.0f, 0.0f
        };
        std::string save_path = "data/frames/";
        std::vector<std::pair<std::string, int>> camera_controls;
    };

    explicit FeatureExtractor(const Config& cfg);
    ~FeatureExtractor();

    bool open();

    // Capture one frame and extract 3 circles sorted by radius (mid, max, min).
    // img_pos[6]: u1,v1, u2,v2, u3,v3
    // rad_out[3]: radius of mid, max, min circle
    // Returns false if capture or detection fails.
    bool extract(float img_pos[6], int rad_out[3]);
    bool extract(float img_pos[6], int rad_out[3], cv::Mat* raw_frame,
                 cv::Mat* annotated_frame);
    bool extract_features(float* img_pos, int img_capacity_points,
                          int* rad_out, int rad_capacity_points,
                          cv::Mat* raw_frame = nullptr,
                          cv::Mat* annotated_frame = nullptr);

private:
    bool extract_features_impl(float* img_pos, int img_capacity_points,
                               int* rad_out, int rad_capacity_points,
                               int feature_count,
                               cv::Mat* raw_frame,
                               cv::Mat* annotated_frame);

    Config        cfg_;
    cv::VideoCapture cap_;
    int           frame_count_ = 0;
};
