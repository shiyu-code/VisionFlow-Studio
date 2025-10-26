#pragma once
#include <opencv2/core.hpp>

struct RdicResult {
    // Displacement fields
    cv::Mat dispX; // float32
    cv::Mat dispY; // float32
    // Strain components
    cv::Mat exx;   // float32
    cv::Mat eyy;   // float32
    cv::Mat exy;   // float32
    // Magnitude (e.g., von Mises-like)
    cv::Mat mag;   // float32
    // Color heatmap (BGR, 8UC3)
    cv::Mat heatmapBGR;
};