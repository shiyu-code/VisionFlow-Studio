#pragma once

#include <vector>
#include <opencv2/core.hpp>

struct PivVector {
    cv::Point2f position;  // center of window (in pixels)
    cv::Point2f displacement; // dx, dy (in pixels), can be scaled later
    float score = 0.0f;    // matching score (e.g., NCC)
};

struct PivResult {
    std::vector<PivVector> vectors; // sparse grid vectors
    cv::Size gridSize;              // number of windows along x/y
};