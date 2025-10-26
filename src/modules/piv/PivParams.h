#pragma once

#include <opencv2/core.hpp>

struct PivParams {
    int windowSize = 32;           // interrogation window size (pixels)
    int overlap = 16;              // overlap between windows (pixels)
    int searchRadius = 8;          // search radius around the window (pixels)
    bool useCLAHE = false;         // apply CLAHE preprocessing
    double calibrationScale = 1.0; // pixels-to-physical scale (units per pixel)
    cv::Rect roi;                  // analysis region (optional); empty => full frame
};