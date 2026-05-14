#pragma once

#include <array>
#include <vector>

#include <opencv2/core.hpp>

struct CircleDetectionConfig {
    double hough_dp       = 1.0;
    double hough_min_dist = 30.0;
    int    hough_param1   = 100;
    int    hough_param2   = 38;
    int    hough_min_rad  = 3;
    int    hough_max_rad  = 80;
    int    blur_kernel    = 7;
    double blur_sigma     = 2.0;
    bool   equalize_hist  = false;
};

struct FeatureCircle {
    cv::Point2f center;
    float       radius = 0.0f;
};

std::vector<FeatureCircle> detect_feature_circles(const cv::Mat& bgr,
                                                  const CircleDetectionConfig& cfg,
                                                  cv::Mat* gray_debug = nullptr);

// Selects the three marker circles and returns them in the controller's legacy
// order: middle radius, largest radius, smallest radius.
bool select_three_feature_circles(const std::vector<FeatureCircle>& circles,
                                  std::array<FeatureCircle, 3>& selected);

void feature_circles_to_arrays(const std::array<FeatureCircle, 3>& selected,
                               float img_pos[6], int rad_out[3]);

void draw_feature_overlay(cv::Mat& image,
                          const std::vector<FeatureCircle>& circles,
                          const std::array<FeatureCircle, 3>* selected,
                          const float desired[6]);
