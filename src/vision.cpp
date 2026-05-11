#include "vision.hpp"
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
}

bool FeatureExtractor::open()
{
    cap_.open(cfg_.camera_index);
    if (!cap_.isOpened()) {
        fprintf(stderr, "FeatureExtractor: cannot open camera %d\n", cfg_.camera_index);
        return false;
    }
    cap_.set(cv::CAP_PROP_FPS, cfg_.fps);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  cfg_.width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cfg_.height);
    return true;
}

bool FeatureExtractor::extract(float img_pos[6], int rad_out[3])
{
    cv::Mat img;
    cap_ >> img;
    if (img.empty()) {
        fprintf(stderr, "FeatureExtractor: empty frame\n");
        return false;
    }

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(7, 7), 2, 2);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, 30,
                     cfg_.hough_param1, cfg_.hough_param2,
                     cfg_.hough_min_rad, cfg_.hough_max_rad);

    if (circles.size() < 3) {
        fprintf(stderr, "FeatureExtractor: detected %zu circles, need 3\n", circles.size());
        return false;
    }

    // Draw detected circles
    for (const auto& c : circles) {
        cv::Point center(cvRound(c[0]), cvRound(c[1]));
        cv::circle(img, center, cvRound(c[2]), cv::Scalar(255, 0, 0), 2);
    }

    // Draw desired positions
    const float* yd = cfg_.desired;
    cv::circle(img, cv::Point(cvRound(yd[0]), cvRound(yd[1])), 21, cv::Scalar(0, 0, 255), 2);
    cv::circle(img, cv::Point(cvRound(yd[2]), cvRound(yd[3])), 24, cv::Scalar(0, 0, 255), 2);
    cv::circle(img, cv::Point(cvRound(yd[4]), cvRound(yd[5])), 17, cv::Scalar(0, 0, 255), 2);

    // Sort first 3 circles by radius to find max, min, mid
    int imax = 0, imin = 0;
    for (int i = 1; i < 3; i++) {
        if (cvRound(circles[i][2]) > cvRound(circles[imax][2])) imax = i;
        if (cvRound(circles[i][2]) < cvRound(circles[imin][2])) imin = i;
    }
    int imid = 3 - imax - imin; // the remaining index

    img_pos[0] = circles[imid][0];
    img_pos[1] = circles[imid][1];
    img_pos[2] = circles[imax][0];
    img_pos[3] = circles[imax][1];
    img_pos[4] = circles[imin][0];
    img_pos[5] = circles[imin][1];

    rad_out[0] = cvRound(circles[imid][2]);
    rad_out[1] = cvRound(circles[imax][2]);
    rad_out[2] = cvRound(circles[imin][2]);

    cv::namedWindow("gu_result", cv::WINDOW_AUTOSIZE);
    cv::imshow("gu_result", img);

    // Save frame
    frame_count_++;
    std::string name = cfg_.save_path + std::to_string(frame_count_) + ".jpg";
    cv::imwrite(name, img);

    cv::waitKey(5);
    return true;
}
