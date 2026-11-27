#include "autocontourdetector.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace {
constexpr double kPi = 3.14159265358979323846;

// 计算图像灰度中位数
double computeMedian(const cv::Mat &gray)
{
    cv::Mat hist;
    int histSize = 256;
    float range[] = {0.0f, 256.0f};
    const float *histRange = {range};
    cv::calcHist(&gray, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);
    float sum = 0.0f;
    const float half = static_cast<float>(gray.total()) * 0.5f;
    for (int i = 0; i < 256; ++i) {
        sum += hist.at<float>(i);
        if (sum >= half) {
            return static_cast<double>(i);
        }
    }
    return 128.0;
}

Detection detectionFromContour(const std::vector<cv::Point> &contour)
{
    Detection d;
    const cv::Moments m = cv::moments(contour);
    if (m.m00 <= 0.0) return d;

    d.area = m.m00;
    d.center = cv::Point2d(m.m10 / m.m00, m.m01 / m.m00);
    d.box = cv::boundingRect(contour);
    d.rotBox = cv::minAreaRect(contour);
    d.aspectRatio = static_cast<double>(d.box.height) / std::max(1, d.box.width);

    // 二阶中心矩求主惯性轴倾角 (-90° ~ +90°)
    const double mu20 = m.mu20 / m.m00;
    const double mu02 = m.mu02 / m.m00;
    const double mu11 = m.mu11 / m.m00;
    const double rad = 0.5 * std::atan2(2.0 * mu11, mu20 - mu02);
    d.angle = rad * (180.0 / kPi);
    const double trace = mu20 + mu02;
    const double separation = std::hypot(mu20 - mu02, 2.0 * mu11);
    d.angleReliable = trace > 0.0 && separation / trace >= 0.20;

    const double boxArea = std::max(1, d.box.area());
    d.score = std::min(1.0, d.area / boxArea);
    d.valid = true;
    return d;
}

cv::Rect clampRect(const cv::Rect &r, const cv::Size &bounds)
{
    return r & cv::Rect(0, 0, bounds.width, bounds.height);
}

cv::Rect expandRect(const cv::Rect &r, int pad, const cv::Size &bounds)
{
    return clampRect(cv::Rect(r.x - pad, r.y - pad, r.width + 2 * pad, r.height + 2 * pad), bounds);
}

cv::Mat computeLabHist(const cv::Mat &bgrPatch)
{
    if (bgrPatch.empty()) return {};
    cv::Mat lab;
    cv::cvtColor(bgrPatch, lab, cv::COLOR_BGR2Lab);
    const int channels[] = {0, 1, 2};
    const int histSize[] = {8, 8, 8};
    const float range[] = {0.0f, 256.0f};
    const float *ranges[] = {range, range, range};
    cv::Mat hist;
    cv::calcHist(&lab, 1, channels, cv::Mat(), hist, 3, histSize, ranges);
    cv::normalize(hist, hist, 1.0, 0.0, cv::NORM_L1);
    return hist;
}

double weightedLabDist(const cv::Vec3b &px, const cv::Vec3d &ref)
{
    const double dL = double(px[0]) - ref[0];
    const double da = double(px[1]) - ref[1];
    const double db = double(px[2]) - ref[2];
    // 降低亮度权重以抑制环境光照微变，同时保留灰度/黑白展品的亮度对比
    return std::sqrt(0.5 * dL * dL + da * da + db * db);
}

bool segmentTargetInBox(const cv::Mat &bgr, const cv::Rect &searchBox, const cv::Vec3d &bgLab,
                        double bgThresh, bool useLab, int minArea, Detection &detOut,
                        std::vector<cv::Point> &contourOut, cv::Mat *patchMaskOut = nullptr)
{
    const cv::Rect box = clampRect(searchBox, bgr.size());
    if (box.width < 8 || box.height < 8) return false;

    const cv::Mat patch = bgr(box);
    cv::Mat mask(box.height, box.width, CV_8UC1, cv::Scalar(0));
    if (useLab) {
        cv::Mat lab;
        cv::cvtColor(patch, lab, cv::COLOR_BGR2Lab);
        const double threshSq = bgThresh * bgThresh;
        for (int r = 0; r < lab.rows; ++r) {
            const auto *src = lab.ptr<cv::Vec3b>(r);
            auto *dst = mask.ptr<uchar>(r);
            for (int c = 0; c < lab.cols; ++c) {
                const double dL = double(src[c][0]) - bgLab[0];
                const double da = double(src[c][1]) - bgLab[1];
                const double db = double(src[c][2]) - bgLab[2];
                dst[c] = (0.5 * dL * dL + da * da + db * db >= threshSq) ? 255 : 0;
            }
        }
        static const cv::Mat kOpen = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        static const cv::Mat kClose = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kOpen);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kClose);
    } else {
        cv::Mat gray, blurred, edges;
        cv::cvtColor(patch, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.1);
        const double med = computeMedian(blurred);
        cv::Canny(blurred, edges, std::max(20.0, med * 0.30), std::min(200.0, med * 0.75));
        static const cv::Mat kClose = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        cv::morphologyEx(edges, mask, cv::MORPH_CLOSE, kClose);
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const cv::Point2d boxCenter(box.width * 0.5, box.height * 0.5);
    const double boxArea = double(box.area());
    const double areaFloor = std::max(double(minArea), boxArea * 0.04);

    double bestScore = -1.0;
    size_t bestIdx = 0;
    bool found = false;
    for (size_t i = 0; i < contours.size(); ++i) {
        const double area = cv::contourArea(contours[i]);
        if (area < areaFloor || area > boxArea * 0.96) continue;
        const cv::Rect cb = cv::boundingRect(contours[i]);
        const cv::Point2d cc(cb.x + cb.width * 0.5, cb.y + cb.height * 0.5);
        const double dist = cv::norm(cc - boxCenter) / std::max(1.0, std::hypot(box.width, box.height));
        if (dist > 0.38 && cv::pointPolygonTest(contours[i], boxCenter, false) < 0) continue;
        const double score = area * (1.0 - 0.8 * dist);
        if (score > bestScore) {
            bestScore = score;
            bestIdx = i;
            found = true;
        }
    }
    if (!found) return false;

    contourOut = contours[bestIdx];
    for (auto &pt : contourOut) {
        pt.x += box.x;
        pt.y += box.y;
    }
    detOut = detectionFromContour(contourOut);
    if (!detOut.valid) return false;
    if (patchMaskOut) *patchMaskOut = mask;
    return true;
}
} // namespace

void AutoContourDetector::pickTarget(int x, int y)
{
    resetTracking();
    m_hasPickedPoint = true;
    m_pickedPoint = cv::Point2d(x, y);
}

void AutoContourDetector::selectRoi(const cv::Rect &roi)
{
    resetTracking();
    m_hasPendingRoi = true;
    m_pendingRoi = roi;
}

void AutoContourDetector::resetTracking()
{
    m_hasPickedPoint = false;
    m_hasPendingRoi = false;
    m_pendingRoi = cv::Rect();
    m_roiMode = false;
    m_referenceFromContour = false;
    m_refTemplate.release();
    m_refHist.release();
    m_bgRingReliable = false;
    m_trackedScale = 1.0;
    m_hasTrackedTarget = false;
    m_trackedArea = 0.0;
    m_reference = Detection();
    m_referenceContour.clear();
}

QList<Detection> AutoContourDetector::initAndDetectRoi(const cv::Mat &bgr)
{
    QList<Detection> result;
    const cv::Rect roi = clampRect(m_pendingRoi, bgr.size());
    m_hasPendingRoi = false;
    if (roi.width < 12 || roi.height < 12) return result;

    // 采样 ROI 外围 12px 环形背景，估计固定机位下的背景 Lab 均值和噪声宽度
    const cv::Rect outer = expandRect(roi, 12, bgr.size());
    cv::Mat outerLab;
    cv::cvtColor(bgr(outer), outerLab, cv::COLOR_BGR2Lab);
    const cv::Rect innerInOuter(roi.x - outer.x, roi.y - outer.y, roi.width, roi.height);

    cv::Vec3d sumLab(0.0, 0.0, 0.0);
    int ringCount = 0;
    std::vector<cv::Vec3b> ringPixels;
    ringPixels.reserve(size_t(outer.area() - roi.area()));
    for (int r = 0; r < outerLab.rows; ++r) {
        const auto *row = outerLab.ptr<cv::Vec3b>(r);
        const bool inY = (r >= innerInOuter.y && r < innerInOuter.y + innerInOuter.height);
        for (int c = 0; c < outerLab.cols; ++c) {
            if (inY && c >= innerInOuter.x && c < innerInOuter.x + innerInOuter.width) continue;
            sumLab[0] += row[c][0];
            sumLab[1] += row[c][1];
            sumLab[2] += row[c][2];
            ringPixels.push_back(row[c]);
            ++ringCount;
        }
    }

    m_bgRingReliable = false;
    m_bgLabSpread = 22.0;
    if (ringCount >= 40) {
        m_bgLabMean = sumLab / double(ringCount);
        std::vector<double> ringDists;
        ringDists.reserve(ringPixels.size());
        for (const auto &px : ringPixels) ringDists.push_back(weightedLabDist(px, m_bgLabMean));
        std::nth_element(ringDists.begin(), ringDists.begin() + ringDists.size() * 3 / 4, ringDists.end());
        const double p75Ring = ringDists[ringDists.size() * 3 / 4];

        std::vector<double> innerDists;
        innerDists.reserve(size_t(roi.area()));
        for (int r = innerInOuter.y; r < innerInOuter.y + innerInOuter.height; ++r) {
            const auto *row = outerLab.ptr<cv::Vec3b>(r);
            for (int c = innerInOuter.x; c < innerInOuter.x + innerInOuter.width; ++c)
                innerDists.push_back(weightedLabDist(row[c], m_bgLabMean));
        }
        std::nth_element(innerDists.begin(), innerDists.begin() + innerDists.size() * 3 / 4, innerDists.end());
        const double p75Inner = innerDists[innerDists.size() * 3 / 4];

        m_bgLabSpread = std::max(18.0, p75Ring * 1.45);
        m_bgRingReliable = (p75Inner > p75Ring * 1.30 + 8.0);
    }

    Detection det;
    std::vector<cv::Point> contour;
    cv::Mat patchMask;
    const int minSegArea = std::max(48, std::min(m_params.minArea, roi.area() / 8));
    bool segmented = segmentTargetInBox(bgr, roi, m_bgLabMean, m_bgLabSpread,
                                        m_bgRingReliable, minSegArea, det, contour, &patchMask);
    if (!segmented && m_bgRingReliable) {
        segmented = segmentTargetInBox(bgr, roi, m_bgLabMean, m_bgLabSpread,
                                       false, minSegArea, det, contour, &patchMask);
    }

    if (segmented && det.area >= std::max(48.0, roi.area() * 0.06)) {
        m_reference = det;
        m_referenceContour = std::move(contour);
        m_referenceFromContour = true;
    } else {
        m_reference = Detection();
        m_reference.box = roi;
        m_reference.rotBox = cv::RotatedRect(
            cv::Point2f(roi.x + roi.width * 0.5f, roi.y + roi.height * 0.5f),
            cv::Size2f(float(roi.width), float(roi.height)), 0.0f);
        m_reference.center = cv::Point2d(roi.x + roi.width * 0.5, roi.y + roi.height * 0.5);
        m_reference.area = double(roi.area());
        m_reference.aspectRatio = double(roi.height) / std::max(1, roi.width);
        m_reference.angle = 0.0;
        m_reference.angleReliable = false;
        m_reference.score = 1.0;
        m_reference.valid = true;
        m_referenceContour.clear();
        m_referenceFromContour = false;
    }

    // 模板外扩 4px 保留目标边界对比度，确保纯色展品也能获得高相关匹配
    const cv::Rect tmplBox = expandRect(m_reference.box, 4, bgr.size());
    cv::cvtColor(bgr(tmplBox), m_refTemplate, cv::COLOR_BGR2GRAY);
    m_refHist = computeLabHist(bgr(m_reference.box));

    m_roiMode = true;
    m_hasTrackedTarget = true;
    m_trackedCenter = m_reference.center;
    m_trackedArea = m_reference.area;
    m_trackedScale = 1.0;

    if (m_computeMask) {
        m_mask = cv::Mat::zeros(bgr.rows, bgr.cols, CV_8UC1);
        if (m_referenceFromContour && !m_referenceContour.empty()) {
            std::vector<std::vector<cv::Point>> draw{m_referenceContour};
            cv::drawContours(m_mask, draw, 0, cv::Scalar(255), cv::FILLED);
        } else {
            cv::rectangle(m_mask, m_reference.box, cv::Scalar(255), cv::FILLED);
        }
    }

    result.append(m_reference);
    return result;
}

QList<Detection> AutoContourDetector::trackRoi(const cv::Mat &bgr)
{
    QList<Detection> result;
    if (m_refTemplate.empty() || !m_reference.valid) return result;

    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    const double ds = (bgr.cols >= 480) ? 0.5 : 1.0;
    cv::Mat graySmall;
    if (ds < 1.0) {
        cv::resize(gray, graySmall, cv::Size(), ds, ds, cv::INTER_AREA);
    } else {
        graySmall = gray;
    }

    static constexpr std::array<double, 9> kScales{0.62, 0.72, 0.82, 0.91, 1.00, 1.09, 1.20, 1.32, 1.46};
    const double diag = std::hypot(m_reference.box.width, m_reference.box.height);

    struct Match {
        double rawCorr = -1.0;
        double score = -2.0;
        double scale = 1.0;
        cv::Rect fullBox;
    };

    const auto searchInWindow = [&](const cv::Rect &fullWin) -> Match {
        Match best;
        const cv::Rect smallWin = clampRect(
            cv::Rect(int(std::floor(fullWin.x * ds)), int(std::floor(fullWin.y * ds)),
                     int(std::ceil(fullWin.width * ds)), int(std::ceil(fullWin.height * ds))),
            graySmall.size());
        if (smallWin.width < 8 || smallWin.height < 8) return best;
        const cv::Mat searchImg = graySmall(smallWin);

        for (double scale : kScales) {
            const int tw = std::max(6, int(std::lround(m_refTemplate.cols * scale * ds)));
            const int th = std::max(6, int(std::lround(m_refTemplate.rows * scale * ds)));
            if (tw >= searchImg.cols || th >= searchImg.rows) continue;

            cv::Mat scaledTmpl;
            cv::resize(m_refTemplate, scaledTmpl, cv::Size(tw, th), 0, 0,
                       scale * ds < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);

            cv::Mat res;
            cv::matchTemplate(searchImg, scaledTmpl, res, cv::TM_CCOEFF_NORMED);
            double maxVal = -1.0;
            cv::Point maxLoc;
            cv::minMaxLoc(res, nullptr, &maxVal, nullptr, &maxLoc);

            const double adj = maxVal - 0.012 * std::abs(scale - m_trackedScale)
                               - 0.005 * std::abs(scale - 1.0);
            if (adj > best.score) {
                best.score = adj;
                best.rawCorr = maxVal;
                best.scale = scale;
                const int fx = int(std::lround((smallWin.x + maxLoc.x) / ds));
                const int fy = int(std::lround((smallWin.y + maxLoc.y) / ds));
                const int fw = int(std::lround(m_refTemplate.cols * scale));
                const int fh = int(std::lround(m_refTemplate.rows * scale));
                best.fullBox = clampRect(cv::Rect(fx, fy, fw, fh), bgr.size());
            }
        }
        return best;
    };

    const int localRadius = int(std::max(140.0, diag * 1.8));
    const cv::Rect localWin = clampRect(
        cv::Rect(int(m_trackedCenter.x) - localRadius, int(m_trackedCenter.y) - localRadius,
                 localRadius * 2, localRadius * 2),
        bgr.size());
    Match best = searchInWindow(localWin);
    if (best.rawCorr < 0.62) {
        const Match globalBest = searchInWindow(cv::Rect(0, 0, bgr.cols, bgr.rows));
        if (globalBest.score > best.score) best = globalBest;
    }

    auto buildMask = [&](const Detection &d, const std::vector<cv::Point> &contour) {
        if (!m_computeMask) return;
        m_mask = cv::Mat::zeros(bgr.rows, bgr.cols, CV_8UC1);
        if (!contour.empty()) {
            std::vector<std::vector<cv::Point>> draw{contour};
            cv::drawContours(m_mask, draw, 0, cv::Scalar(255), cv::FILLED);
        } else if (d.valid) {
            cv::rectangle(m_mask, d.box, cv::Scalar(255), cv::FILLED);
        }
    };

    if (best.rawCorr >= 0.52 && best.fullBox.width >= 8 && best.fullBox.height >= 8) {
        // 去掉 4px 模板边界留白还原紧致外接框
        const int margin = int(std::lround(4.0 * best.scale));
        cv::Rect tightBox = clampRect(
            cv::Rect(best.fullBox.x + margin, best.fullBox.y + margin,
                     std::max(6, best.fullBox.width - 2 * margin),
                     std::max(6, best.fullBox.height - 2 * margin)),
            bgr.size());
        const cv::Mat currHist = computeLabHist(bgr(tightBox));
        const double histCorr = (m_refHist.empty() || currHist.empty())
                                    ? 1.0
                                    : cv::compareHist(m_refHist, currHist, cv::HISTCMP_CORREL);
        if (histCorr >= 0.35) {
            Detection out;
            std::vector<cv::Point> contour;
            bool refined = false;
            if (m_referenceFromContour) {
                const cv::Rect segWin = expandRect(tightBox, 10, bgr.size());
                const int minSegArea = std::max(36, int(m_reference.area * 0.25));
                refined = segmentTargetInBox(bgr, segWin, m_bgLabMean, m_bgLabSpread,
                                             m_bgRingReliable, minSegArea, out, contour);
                if (refined) {
                    const double expectedArea = m_reference.area * best.scale * best.scale;
                    const double ratio = out.area / std::max(1.0, expectedArea);
                    if (ratio < 0.45 || ratio > 2.1) refined = false;
                }
            }
            if (!refined) {
                out = Detection();
                out.box = tightBox;
                out.rotBox = cv::RotatedRect(
                    cv::Point2f(tightBox.x + tightBox.width * 0.5f, tightBox.y + tightBox.height * 0.5f),
                    cv::Size2f(float(tightBox.width), float(tightBox.height)), 0.0f);
                out.center = cv::Point2d(tightBox.x + tightBox.width * 0.5, tightBox.y + tightBox.height * 0.5);
                out.area = m_reference.area * best.scale * best.scale;
                out.aspectRatio = double(tightBox.height) / std::max(1, tightBox.width);
                out.angle = m_reference.angle;
                out.angleReliable = false;
                out.score = std::clamp(best.rawCorr, 0.0, 1.0);
                out.valid = true;
                contour.clear();
            }
            m_trackedCenter = out.center;
            m_trackedArea = out.area;
            m_trackedScale = best.scale;
            buildMask(out, contour);
            result.append(out);
            return result;
        }
    }

    // 兜底：若目标原地发生较大幅度倾倒/旋转（模板相关性下降但前景仍在原位），用前景分割捕捉倾斜
    if (m_bgRingReliable && m_referenceFromContour) {
        const int pad = int(std::max(30.0, diag * 0.55));
        const cv::Rect aroundRef = expandRect(m_reference.box, pad, bgr.size());
        Detection rotated;
        std::vector<cv::Point> contour;
        const int minSegArea = std::max(48, int(m_reference.area * 0.35));
        if (segmentTargetInBox(bgr, aroundRef, m_bgLabMean, m_bgLabSpread, true, minSegArea, rotated, contour)) {
            const double areaRatio = rotated.area / std::max(1.0, m_reference.area);
            const cv::Mat currHist = computeLabHist(bgr(clampRect(rotated.box, bgr.size())));
            const double histCorr = (m_refHist.empty() || currHist.empty())
                                        ? 1.0
                                        : cv::compareHist(m_refHist, currHist, cv::HISTCMP_CORREL);
            if (areaRatio >= 0.40 && areaRatio <= 2.40 && histCorr >= 0.50) {
                m_trackedCenter = rotated.center;
                m_trackedArea = rotated.area;
                m_trackedScale = std::sqrt(areaRatio);
                buildMask(rotated, contour);
                result.append(rotated);
                return result;
            }
        }
    }

    if (m_computeMask) m_mask = cv::Mat::zeros(bgr.rows, bgr.cols, CV_8UC1);
    return result;
}

QList<Detection> AutoContourDetector::detect(const cv::Mat &bgr)
{
    QList<Detection> result;
    if (bgr.empty() || bgr.type() != CV_8UC3) {
        m_mask.release();
        return result;
    }

    if (m_hasPendingRoi) {
        return initAndDetectRoi(bgr);
    }
    if (m_roiMode) {
        return trackRoi(bgr);
    }

    // 无人工选择且无需显示掩膜时直接跳过全帧边缘提取，降低三路 CPU 占用
    if (!m_hasPickedPoint && !m_hasTrackedTarget && !m_computeMask) {
        m_mask.release();
        return result;
    }

    // 1. 灰度化与高斯降噪
    cv::Mat gray, blurred;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.2);

    // 2. 自适应边缘提取（针对真实光照场景）
    cv::Mat edges;
    if (m_params.autoThreshold) {
        const double med = computeMedian(blurred);
        const double lowThresh = std::max(20.0, med * 0.30);
        const double highThresh = std::min(200.0, med * 0.75);
        cv::Canny(blurred, edges, lowThresh, highThresh);
    } else {
        cv::Canny(blurred, edges, m_params.cannyLow, m_params.cannyHigh);
    }

    // 3. 形态学闭运算，将物体轮廓边缘断线闭合
    const int kSize = std::max(3, m_params.morphKernelSize | 1);
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kSize, kSize));
    cv::morphologyEx(edges, m_mask, cv::MORPH_CLOSE, kernel);

    // 4. 提取外轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(m_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 5. 无人工选择时只生成掩膜，不自动选择最大的背景轮廓。
    if (!m_hasPickedPoint && !m_hasTrackedTarget)
        return result;
    const bool picking = m_hasPickedPoint;
    m_hasPickedPoint = false; // 空白处点击也只尝试本帧，不留下隐含的未来选择。
    struct Candidate {
        double score = 0.0;
        Detection det;
        size_t contourIndex = 0;
    };
    std::vector<Candidate> candidates;

    for (size_t i = 0; i < contours.size(); ++i) {
        const auto &contour = contours[i];
        const double area = cv::contourArea(contour);
        if (area < std::max(1, m_params.minArea)) {
            continue;
        }

        Detection d = detectionFromContour(contour);
        if (!d.valid) continue;

        double score = 0.0;
        if (picking) {
            // 必须点在轮廓内部。不能用最近的大背景替代一次失败的选点。
            if (cv::pointPolygonTest(contour, m_pickedPoint, false) < 0)
                continue;
            score = 1.0 / d.area;
        } else {
            const double diagonal = std::hypot(m_reference.box.width, m_reference.box.height);
            const double referenceDistance = cv::norm(d.center - m_reference.center);
            const double distToLast = cv::norm(d.center - m_trackedCenter);
            const double areaRatio = d.area / std::max(1.0, m_reference.area);
            const double shapeDistance = cv::matchShapes(m_referenceContour, contour, cv::CONTOURS_MATCH_I1, 0);
            // 以人工选择时的目标为锚，避免逐帧软评分慢慢漂移到其他目标。
            if (referenceDistance > std::max(30.0, diagonal * 0.8)
                || distToLast > std::max(25.0, diagonal * 0.5)
                || areaRatio < 0.4 || areaRatio > 2.5 || shapeDistance > 0.30)
                continue;
            score = 1.0 / (1.0 + distToLast / std::max(1.0, diagonal)
                           + std::abs(std::log(areaRatio)) + shapeDistance);
        }

        candidates.push_back({score, d, i});
    }

    if (candidates.empty()) {
        // 保留原身份约束：下一帧仍只寻找原目标，不重新选择背景。
        return result;
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &a, const Candidate &b) { return a.score > b.score; });

    // 锁定主追踪目标
    const Detection &best = candidates.front().det;
    if (picking) {
        m_reference = best;
        m_referenceContour = contours[candidates.front().contourIndex];
    }
    m_trackedCenter = best.center;
    m_trackedArea = best.area;
    m_hasTrackedTarget = true;
    result.append(best); // 当前流程仅监测一个人工指定目标。

    return result;
}
