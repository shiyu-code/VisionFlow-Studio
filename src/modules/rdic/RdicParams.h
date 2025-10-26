#pragma once
#include <opencv2/core.hpp>

struct RdicParams {
    int windowSize = 32;   // subset size for local strain estimation (pixels)
    int gridStep = 16;     // sampling grid step (pixels)
    double smoothSigma = 1.0; // Gaussian smoothing for displacement
};