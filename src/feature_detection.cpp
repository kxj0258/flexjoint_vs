#include "feature_detection.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

std::vector<FeatureCircle> detect_feature_circles(const cv::Mat& bgr,
                                                  const CircleDetectionConfig& cfg,
                                                  cv::Mat* gray_debug)
{
    cv::Mat gray;
    if (bgr.channels() == 1)
        gray = bgr.clone();
    else
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    if (cfg.equalize_hist)
        cv::equalizeHist(gray, gray);

    int blur_kernel = std::max(1, cfg.blur_kernel);
    if (blur_kernel % 2 == 0)
        ++blur_kernel;
    if (blur_kernel > 1)
        cv::GaussianBlur(gray, gray, cv::Size(blur_kernel, blur_kernel),
                         cfg.blur_sigma, cfg.blur_sigma);

    if (gray_debug)
        *gray_debug = gray.clone();

    std::vector<cv::Vec3f> raw_circles;
    cv::HoughCircles(gray, raw_circles, cv::HOUGH_GRADIENT,
                     std::max(0.1, cfg.hough_dp),
                     std::max(1.0, cfg.hough_min_dist),
                     std::max(1, cfg.hough_param1),
                     std::max(1, cfg.hough_param2),
                     std::max(0, cfg.hough_min_rad),
                     std::max(0, cfg.hough_max_rad));

    std::vector<FeatureCircle> circles;
    circles.reserve(raw_circles.size());
    for (const auto& c : raw_circles)
        circles.push_back({cv::Point2f(c[0], c[1]), c[2]});
    return circles;
}

bool select_three_feature_circles(const std::vector<FeatureCircle>& circles,
                                  std::array<FeatureCircle, 3>& selected)
{
    if (circles.size() < 3)
        return false;

    std::vector<FeatureCircle> sorted = circles;
    std::sort(sorted.begin(), sorted.end(),
              [](const FeatureCircle& a, const FeatureCircle& b) {
                  return a.radius < b.radius;
              });

    const FeatureCircle& min_circle = sorted.front();
    const FeatureCircle& mid_circle = sorted[sorted.size() / 2];
    const FeatureCircle& max_circle = sorted.back();

    selected[0] = mid_circle;
    selected[1] = max_circle;
    selected[2] = min_circle;
    return true;
}

void feature_circles_to_arrays(const std::array<FeatureCircle, 3>& selected,
                               float img_pos[6], int rad_out[3])
{
    for (int i = 0; i < 3; ++i) {
        img_pos[2 * i]     = selected[i].center.x;
        img_pos[2 * i + 1] = selected[i].center.y;
        rad_out[i]         = cvRound(selected[i].radius);
    }
}

void draw_feature_overlay(cv::Mat& image,
                          const std::vector<FeatureCircle>& circles,
                          const std::array<FeatureCircle, 3>* selected,
                          const float desired[6])
{
    (void)circles;

    if (selected) {
        for (int i = 0; i < 3; ++i) {
            const auto& c = (*selected)[i];
            cv::Point center(cvRound(c.center.x), cvRound(c.center.y));
            cv::circle(image, center, cvRound(c.radius), cv::Scalar(255, 0, 0), 2);
        }
    }

    if (desired) {
        static const int desired_radius[3] = {21, 24, 17};
        for (int i = 0; i < 3; ++i) {
            cv::circle(image,
                       cv::Point(cvRound(desired[2 * i]), cvRound(desired[2 * i + 1])),
                       desired_radius[i], cv::Scalar(0, 0, 255), 2);
        }
    }
}
