#include "vision.hpp"
#include "camera_utils.hpp"
#include "feature_detection.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cstdio>
#include <string>
#include <vector>
#include <algorithm>

FeatureExtractor::FeatureExtractor(const Config& cfg) : cfg_(cfg) {}

FeatureExtractor::~FeatureExtractor()
{
    if (cap_.isOpened())
        cap_.release();
    cv::destroyWindow("camera_raw");
    cv::destroyWindow("gu_result");
}

bool FeatureExtractor::open()
{
    return open_configured_camera(cap_, cfg_, "FeatureExtractor");
}

bool FeatureExtractor::extract(float img_pos[6], int rad_out[3])
{
    cv::Mat img;
    cap_ >> img;
    if (img.empty()) {
        fprintf(stderr, "FeatureExtractor: empty frame\n");
        return false;
    }

    const cv::Mat raw_view = img.clone();
    cv::namedWindow("camera_raw", cv::WINDOW_AUTOSIZE);
    cv::imshow("camera_raw", raw_view);

    CircleDetectionConfig detect_cfg;
    detect_cfg.hough_dp       = cfg_.hough_dp;
    detect_cfg.hough_min_dist = cfg_.hough_min_dist;
    detect_cfg.hough_param1   = cfg_.hough_param1;
    detect_cfg.hough_param2   = cfg_.hough_param2;
    detect_cfg.hough_min_rad  = cfg_.hough_min_rad;
    detect_cfg.hough_max_rad  = cfg_.hough_max_rad;
    detect_cfg.blur_kernel    = cfg_.blur_kernel;
    detect_cfg.blur_sigma     = cfg_.blur_sigma;
    detect_cfg.equalize_hist  = cfg_.equalize_hist;

    std::vector<FeatureCircle> circles = detect_feature_circles(img, detect_cfg);
    if (circles.size() < 3) {
        cv::namedWindow("gu_result", cv::WINDOW_AUTOSIZE);
        cv::imshow("gu_result", img);
        cv::waitKey(5);
        fprintf(stderr, "FeatureExtractor: detected %zu circles, need 3\n", circles.size());
        return false;
    }

    std::array<FeatureCircle, 3> selected;
    if (!select_three_feature_circles(circles, selected)) {
        cv::namedWindow("gu_result", cv::WINDOW_AUTOSIZE);
        cv::imshow("gu_result", img);
        cv::waitKey(5);
        fprintf(stderr, "FeatureExtractor: cannot select 3 feature circles\n");
        return false;
    }
    feature_circles_to_arrays(selected, img_pos, rad_out);
    draw_feature_overlay(img, circles, &selected, cfg_.desired);

    cv::namedWindow("gu_result", cv::WINDOW_AUTOSIZE);
    cv::imshow("gu_result", img);

    // Save frame
    frame_count_++;
    std::string name = cfg_.save_path + std::to_string(frame_count_) + ".jpg";
    cv::imwrite(name, img);

    cv::waitKey(5);
    return true;
}
