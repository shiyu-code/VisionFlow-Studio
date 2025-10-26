#pragma once
#include "RdicParams.h"
#include "RdicResult.h"
#include <opencv2/core.hpp>

namespace RdicEngine {
    // Basic RDIC: approximate displacement via Farneback optical flow,
    // compute strain via spatial gradients, and generate a heatmap.
    bool analyze(const cv::Mat& img0, const cv::Mat& img1, const RdicParams& params, RdicResult& out);
}