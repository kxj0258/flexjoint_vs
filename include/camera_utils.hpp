#pragma once

#include <string>

#include <opencv2/videoio.hpp>

#include "vision.hpp"

bool open_configured_camera(cv::VideoCapture& cap,
                            const FeatureExtractor::Config& cfg,
                            const char* owner,
                            bool verbose = true);

bool apply_camera_controls(const FeatureExtractor::Config& cfg,
                           const char* owner,
                           bool verbose = true);
