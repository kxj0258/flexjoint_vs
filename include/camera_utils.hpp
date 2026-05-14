#pragma once

#include <opencv2/videoio.hpp>

#include "vision.hpp"

bool open_configured_camera(cv::VideoCapture& cap,
                            const FeatureExtractor::Config& cfg,
                            const char* owner,
                            bool verbose = true);
