#pragma once

#include <opencv2/core.hpp>
#include "PivParams.h"
#include "PivResult.h"

class PivEngine {
public:
    // Perform a simple block-matching PIV between two grayscale frames
    // Returns true on success, fills outResult
    static bool analyze(const cv::Mat& frame0, const cv::Mat& frame1,
                        const PivParams& params, PivResult& outResult);
};