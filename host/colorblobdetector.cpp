#include "colorblobdetector.h"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace {
constexpr double kPi = 3.14159265358979323846;

int clampInt(int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); }
} // namespace

cv::Mat ColorBlobDetector::thresholdMask(const cv::Mat &hsv, const ColorParams &p)
{
    cv::Mat mask;
    const int hMin = clampInt(p.hMin, 0, 179), hMax = clampInt(p.hMax, 0, 179);
    if (hMin <= hMax) {
        cv::inRange(hsv, cv::Scalar(hMin, p.sMin, p.vMin), cv::Scalar(hMax, p.sMax, p.vMax), mask);
    } else {
        // 色相跨过 0：分成 [hMin,179] 和 [0,hMax] 两段
        cv::Mat a, b;
        cv::inRange(hsv, cv::Scalar(hMin, p.sMin, p.vMin), cv::Scalar(179, p.sMax, p.vMax), a);
        cv::inRange(hsv, cv::Scalar(0, p.sMin, p.vMin), cv::Scalar(hMax, p.sMax, p.vMax), b);
        cv::bitwise_or(a, b, mask);
    }
    return mask;
}

QList<Detection> ColorBlobDetector::detect(const cv::Mat &bgr)
{
    QList<Detection> result;
    if (bgr.empty() || bgr.type() != CV_8UC3) {
        m_mask.release();
        return result;
    }

    cv::cvtColor(bgr, m_hsv, cv::COLOR_BGR2HSV);
    m_mask = thresholdMask(m_hsv, m_params);

    // 开运算去掉孤立噪点，闭运算填补色块内的小洞
    static const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(m_mask, m_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(m_mask, m_mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    // OpenCV >= 3.2 保留输入掩膜，可直接供显示使用。
    cv::findContours(m_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto &contour : contours) {
        const cv::Moments m = cv::moments(contour);
        if (m.m00 < std::max(1, m_params.minArea))
            continue;
        Detection d;
        d.area = m.m00;
        d.center = cv::Point2d(m.m10 / m.m00, m.m01 / m.m00);
        d.box = cv::boundingRect(contour);
        d.rotBox = cv::minAreaRect(contour);
        d.aspectRatio = static_cast<double>(d.box.height) / std::max(1, d.box.width);

        // 二阶中心矩计算主惯性轴方向角
        const double mu20 = m.mu20 / m.m00;
        const double mu02 = m.mu02 / m.m00;
        const double mu11 = m.mu11 / m.m00;
        const double rad = 0.5 * std::atan2(2.0 * mu11, mu20 - mu02);
        d.angle = rad * (180.0 / kPi);
        const double trace = mu20 + mu02;
        d.angleReliable = trace > 0.0 && std::hypot(mu20 - mu02, 2.0 * mu11) / trace >= 0.20;

        // 轮廓面积 / 外接矩形面积：越接近 1 越"实"，用作简单的可信度
        const double boxArea = std::max(1, d.box.area());
        d.score = std::min(1.0, d.area / boxArea);
        d.valid = true;
        result.append(d);
    }

    std::sort(result.begin(), result.end(),
              [](const Detection &a, const Detection &b) { return a.area > b.area; });
    const int limit = std::max(1, m_params.maxTargets);
    if (result.size() > limit)
        result.erase(result.begin() + limit, result.end());
    return result;
}

bool ColorBlobDetector::pickColor(const cv::Mat &bgr, int x, int y, ColorParams *params, int radius)
{
    if (bgr.empty() || bgr.type() != CV_8UC3 || x < 0 || y < 0 || x >= bgr.cols || y >= bgr.rows)
        return false;

    const cv::Rect roi = cv::Rect(x - radius, y - radius, 2 * radius + 1, 2 * radius + 1)
                         & cv::Rect(0, 0, bgr.cols, bgr.rows);
    cv::Mat hsv;
    cv::cvtColor(bgr(roi), hsv, cv::COLOR_BGR2HSV);

    // 色相是环形量（0 与 179 相邻），用圆周平均
    double sx = 0, sy = 0;
    int sMin = 255, vMin = 255;
    std::vector<double> hues;
    hues.reserve(hsv.total());
    for (int r = 0; r < hsv.rows; ++r) {
        for (int c = 0; c < hsv.cols; ++c) {
            const cv::Vec3b px = hsv.at<cv::Vec3b>(r, c);
            const double angle = px[0] * 2.0 * kPi / 180.0;
            sx += std::cos(angle);
            sy += std::sin(angle);
            hues.push_back(px[0]);
            sMin = std::min<int>(sMin, px[1]);
            vMin = std::min<int>(vMin, px[2]);
        }
    }
    double meanAngle = std::atan2(sy, sx);
    if (meanAngle < 0)
        meanAngle += 2.0 * kPi;
    const int hMean = int(std::lround(meanAngle * 180.0 / (2.0 * kPi))) % 180;

    // 邻域内与平均色相的最大环形距离
    int spread = 0;
    for (double h : hues) {
        int d = std::abs(int(h) - hMean);
        d = std::min(d, 180 - d);
        spread = std::max(spread, d);
    }
    const int tol = clampInt(spread + 8, 8, 25);

    params->hMin = (hMean - tol + 180) % 180;
    params->hMax = (hMean + tol) % 180;
    params->sMin = clampInt(sMin - 40, 50, 255);
    params->sMax = 255;
    params->vMin = clampInt(vMin - 50, 30, 255);
    params->vMax = 255;
    return true;
}
