#include "PivEngine.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace {
    cv::Mat ensureGray8(const cv::Mat& src) {
        cv::Mat gray;
        if (src.channels() == 1) {
            if (src.type() != CV_8U) src.convertTo(gray, CV_8U);
            else gray = src;
        } else {
            cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        }
        return gray;
    }

    void applyCLAHEIfNeeded(cv::Mat& img, bool useCLAHE) {
        if (!useCLAHE) return;
        auto clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
        clahe->apply(img, img);
    }
}

bool PivEngine::analyze(const cv::Mat& frame0, const cv::Mat& frame1,
                        const PivParams& params, PivResult& outResult) {
    if (frame0.empty() || frame1.empty()) return false;
    cv::Mat f0 = ensureGray8(frame0);
    cv::Mat f1 = ensureGray8(frame1);

    applyCLAHEIfNeeded(f0, params.useCLAHE);
    applyCLAHEIfNeeded(f1, params.useCLAHE);

    cv::Rect roi = params.roi;
    if (roi.width <= 0 || roi.height <= 0) {
        roi = cv::Rect(0, 0, std::min(f0.cols, f1.cols), std::min(f0.rows, f1.rows));
    }

    int W = params.windowSize;
    int O = std::clamp(params.overlap, 0, std::max(0, W - 1));
    int step = std::max(1, W - O);
    int R = std::max(0, params.searchRadius);

    int gx = (roi.width - W) / step + 1;
    int gy = (roi.height - W) / step + 1;
    if (gx <= 0 || gy <= 0) return false;

    outResult.vectors.clear();
    outResult.vectors.reserve(static_cast<size_t>(gx * gy));
    outResult.gridSize = cv::Size(gx, gy);

    for (int j = 0; j < gy; ++j) {
        for (int i = 0; i < gx; ++i) {
            int x0 = roi.x + i * step;
            int y0 = roi.y + j * step;
            cv::Rect win0(x0, y0, W, W);
            cv::Mat tile0 = f0(win0);

            // Search region in frame1
            int sx = std::max(roi.x, x0 - R);
            int sy = std::max(roi.y, y0 - R);
            int ex = std::min(roi.x + roi.width - W, x0 + R);
            int ey = std::min(roi.y + roi.height - W, y0 + R);
            int sw = ex - sx + W;
            int sh = ey - sy + W;
            if (sw <= W || sh <= W) {
                // not enough area to search
                continue;
            }
            cv::Rect searchRect(sx, sy, sw, sh);
            cv::Mat search1 = f1(searchRect);

            cv::Mat result;
            int method = cv::TM_CCOEFF_NORMED;
            cv::matchTemplate(search1, tile0, result, method);
            double minVal, maxVal;
            cv::Point minLoc, maxLoc;
            cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

            cv::Point best = maxLoc; // position in search1 where tile0 best matches
            // Convert to displacement relative to win0 top-left
            cv::Point2f disp((float)(searchRect.x + best.x - x0), (float)(searchRect.y + best.y - y0));

            PivVector vec;
            vec.position = cv::Point2f(x0 + W * 0.5f, y0 + W * 0.5f);
            vec.displacement = disp;
            vec.score = static_cast<float>(maxVal);
            outResult.vectors.push_back(vec);
        }
    }

    // Optional: apply calibration scale to convert to physical units
    if (params.calibrationScale != 1.0) {
        for (auto& v : outResult.vectors) {
            v.displacement *= static_cast<float>(params.calibrationScale);
        }
    }

    return !outResult.vectors.empty();
}