#include "RdicEngine.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <algorithm>

namespace {
static cv::Mat toGray(const cv::Mat& src) {
    if (src.channels() == 1) return src;
    cv::Mat g;
    cv::cvtColor(src, g, cv::COLOR_BGR2GRAY);
    return g;
}

static void computeStrain(const cv::Mat& Ux, const cv::Mat& Uy,
                          cv::Mat& exx, cv::Mat& eyy, cv::Mat& exy) {
    cv::Mat dUx_dx, dUx_dy, dUy_dx, dUy_dy;
    cv::Sobel(Ux, dUx_dx, CV_32F, 1, 0, 3);
    cv::Sobel(Ux, dUx_dy, CV_32F, 0, 1, 3);
    cv::Sobel(Uy, dUy_dx, CV_32F, 1, 0, 3);
    cv::Sobel(Uy, dUy_dy, CV_32F, 0, 1, 3);
    exx = dUx_dx;
    eyy = dUy_dy;
    exy = 0.5f * (dUx_dy + dUy_dx);
}

static cv::Mat makeHeatmap(const cv::Mat& mag) {
    double minVal=0.0, maxVal=1.0;
    cv::minMaxLoc(mag, &minVal, &maxVal);
    double eps = 1e-9;
    double range = std::max(maxVal - minVal, eps);
    cv::Mat norm;
    mag.convertTo(norm, CV_32F);
    norm = (norm - static_cast<float>(minVal)) * (1.0f/static_cast<float>(range));
    cv::Mat u8;
    norm.convertTo(u8, CV_8U, 255.0);
    cv::Mat heat;
    cv::applyColorMap(u8, heat, cv::COLORMAP_JET);
    return heat; // BGR
}
}

namespace RdicEngine {

bool analyze(const cv::Mat& img0, const cv::Mat& img1, const RdicParams& params, RdicResult& out) {
    if (img0.empty() || img1.empty()) return false;
    cv::Mat g0 = toGray(img0);
    cv::Mat g1 = toGray(img1);

    // Optical flow (Farneback) as approximate displacement
    cv::Mat flow; // 2-channel float: (Ux, Uy)
    try {
        cv::calcOpticalFlowFarneback(g0, g1, flow,
            0.5, 3, params.windowSize, 3, 5, 1.2, 0);
    } catch (...) {
        return false;
    }

    std::vector<cv::Mat> ch(2);
    cv::split(flow, ch);
    out.dispX = ch[0];
    out.dispY = ch[1];

    if (params.smoothSigma > 0.01) {
        cv::GaussianBlur(out.dispX, out.dispX, cv::Size(0,0), params.smoothSigma);
        cv::GaussianBlur(out.dispY, out.dispY, cv::Size(0,0), params.smoothSigma);
    }

    computeStrain(out.dispX, out.dispY, out.exx, out.eyy, out.exy);

    // Von Mises-like scalar for 2D strain
    cv::Mat mag;
    mag = out.exx.mul(out.exx) + out.eyy.mul(out.eyy) + 2.0f*out.exy.mul(out.exy);
    cv::sqrt(mag, mag);
    out.mag = mag;
    out.heatmapBGR = makeHeatmap(out.mag);
    return true;
}

}