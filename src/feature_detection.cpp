#include "feature_detection.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace {

bool valid_feature_count(int feature_count)
{
    return feature_count >= kMinFeaturePoints &&
           feature_count <= kMaxFeaturePoints;
}

bool circle_center_less(const FeatureCircle& a, const FeatureCircle& b)
{
    if (a.center.x != b.center.x)
        return a.center.x < b.center.x;
    return a.center.y < b.center.y;
}

bool radius_asc_then_center(const FeatureCircle& a, const FeatureCircle& b)
{
    if (a.radius != b.radius)
        return a.radius < b.radius;
    return circle_center_less(a, b);
}

} // namespace

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

bool select_feature_circles(const std::vector<FeatureCircle>& circles,
                            int feature_count,
                            std::array<FeatureCircle, kMaxFeaturePoints>& selected)
{
    if (!valid_feature_count(feature_count) ||
        circles.size() < static_cast<size_t>(feature_count)) {
        return false;
    }

    if (feature_count == kLegacyFeaturePoints) {
        std::array<FeatureCircle, kLegacyFeaturePoints> legacy;
        if (!select_three_feature_circles(circles, legacy))
            return false;
        for (int i = 0; i < kLegacyFeaturePoints; ++i)
            selected[i] = legacy[i];
        return true;
    }

    std::vector<FeatureCircle> sorted = circles;
    std::sort(sorted.begin(), sorted.end(), radius_asc_then_center);

    if (feature_count == kMinFeaturePoints) {
        selected[0] = sorted.front(); // smallest radius
        selected[1] = sorted.back();  // largest radius
        return true;
    }

    selected[0] = sorted[1];                 // second-smallest radius
    selected[1] = sorted[sorted.size() - 2]; // second-largest radius
    selected[2] = sorted.front();            // smallest radius
    selected[3] = sorted.back();             // largest radius
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

bool feature_circles_to_arrays(
    const std::array<FeatureCircle, kMaxFeaturePoints>& selected,
    int feature_count,
    float* img_pos,
    int img_capacity_points,
    int* rad_out,
    int rad_capacity_points)
{
    if (!valid_feature_count(feature_count) || !img_pos || !rad_out ||
        img_capacity_points < feature_count ||
        rad_capacity_points < feature_count) {
        return false;
    }

    for (int i = 0; i < feature_count; ++i) {
        img_pos[2 * i]     = selected[i].center.x;
        img_pos[2 * i + 1] = selected[i].center.y;
        rad_out[i]         = cvRound(selected[i].radius);
    }
    return true;
}

void draw_feature_overlay(cv::Mat& image,
                          const std::vector<FeatureCircle>& circles,
                          const std::array<FeatureCircle, 3>* selected,
                          const float desired[6])
{
    std::array<FeatureCircle, kMaxFeaturePoints> selected_max;
    if (selected) {
        for (int i = 0; i < kLegacyFeaturePoints; ++i)
            selected_max[i] = (*selected)[i];
    }
    draw_feature_overlay(image, circles, selected ? &selected_max : nullptr,
                         kLegacyFeaturePoints, desired);
}

void draw_feature_overlay(
    cv::Mat& image,
    const std::vector<FeatureCircle>& circles,
    const std::array<FeatureCircle, kMaxFeaturePoints>* selected,
    int feature_count,
    const float* desired)
{
    (void)circles;

    if (selected) {
        for (int i = 0; i < feature_count; ++i) {
            const auto& c = (*selected)[i];
            cv::Point center(cvRound(c.center.x), cvRound(c.center.y));
            cv::circle(image, center, cvRound(c.radius), cv::Scalar(255, 0, 0), 2);
        }
    }

    if (desired) {
        static const int legacy_desired_radius[kLegacyFeaturePoints] = {
            21, 24, 17
        };
        for (int i = 0; i < feature_count; ++i) {
            int desired_radius = 18;
            if (feature_count == kLegacyFeaturePoints)
                desired_radius = legacy_desired_radius[i];
            else if (selected)
                desired_radius = std::max(3, cvRound((*selected)[i].radius));
            cv::circle(image,
                       cv::Point(cvRound(desired[2 * i]), cvRound(desired[2 * i + 1])),
                       desired_radius, cv::Scalar(0, 0, 255), 2);
        }
    }
}
