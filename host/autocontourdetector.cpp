#include "autocontourdetector.h"
#include "artifactmonitor.h"
#include "yolodetector.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr int kSatFloor = 38;
constexpr int kValFloor = 35;
constexpr int kUnreliableHoldLimit = 3;
constexpr double kHardRejectIdentityScore = 0.14;
constexpr double kSizeDeadZoneRatio = 0.055;
constexpr double kSizeDeadZonePixels = 5.0;
constexpr qint64 kYoloAnchorIntervalMs = 450;
constexpr int kOrbMinGoodMatches = 8;
constexpr int kOrbMinInliers = 6;

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
        if (sum >= half) return static_cast<double>(i);
    }
    return 128.0;
}

double medianDouble(std::vector<double> values)
{
    if (values.empty()) return 0.0;
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    double val = values[mid];
    if (values.size() % 2 == 0) {
        std::nth_element(values.begin(), values.begin() + mid - 1, values.end());
        val = (val + values[mid - 1]) * 0.5;
    }
    return val;
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

// 收缩框选四边，尽量剔除目标外接矩形四角的背景像素
cv::Rect innerCoreRect(const cv::Rect &r, const cv::Size &bounds)
{
    const int dx = std::max(0, int(std::floor(r.width * 0.14)));
    const int dy = std::max(0, int(std::floor(r.height * 0.14)));
    if (r.width - 2 * dx < 6 || r.height - 2 * dy < 6)
        return clampRect(r, bounds);
    return clampRect(cv::Rect(r.x + dx, r.y + dy, r.width - 2 * dx, r.height - 2 * dy), bounds);
}

// 旋转无关形状相似度（借鉴 untitled7::featureShapeSimilarity，兼容 90° 倾倒同时拒绝小块噪点）
double rotationInvariantShapeSimilarity(const cv::Size &cand, const cv::Size &ref)
{
    if (cand.width < 8 || cand.height < 8 || ref.width < 8 || ref.height < 8) return 0.0;
    const double cMax = double(std::max(cand.width, cand.height));
    const double cMin = double(std::min(cand.width, cand.height));
    const double rMax = double(std::max(ref.width, ref.height));
    const double rMin = double(std::min(ref.width, ref.height));
    const double maxSim = std::min(cMax, rMax) / std::max(cMax, rMax);
    const double minSim = std::min(cMin, rMin) / std::max(cMin, rMin);
    const double cElong = cMax / std::max(1.0, cMin);
    const double rElong = rMax / std::max(1.0, rMin);
    const double elongSim = std::min(cElong, rElong) / std::max(cElong, rElong);
    return std::clamp(0.35 * maxSim + 0.35 * minSim + 0.30 * elongSim, 0.0, 1.0);
}

double rectIoU(const cv::Rect &a, const cv::Rect &b)
{
    const double inter = double((a & b).area());
    if (inter <= 0.0) return 0.0;
    const double uni = double(a.area()) + double(b.area()) - inter;
    return uni > 0.0 ? (inter / uni) : 0.0;
}

double weightedLabDist(const cv::Vec3b &px, const cv::Vec3d &ref)
{
    const double dL = double(px[0]) - ref[0];
    const double da = double(px[1]) - ref[1];
    const double db = double(px[2]) - ref[2];
    return std::sqrt(0.5 * dL * dL + da * da + db * db);
}

cv::Mat computeLabHist(const cv::Mat &bgrPatch, const cv::Mat &mask = cv::Mat())
{
    if (bgrPatch.empty()) return {};
    cv::Mat lab;
    cv::cvtColor(bgrPatch, lab, cv::COLOR_BGR2Lab);
    const int channels[] = {0, 1, 2};
    const int histSize[] = {8, 8, 8};
    const float range[] = {0.0f, 256.0f};
    const float *ranges[] = {range, range, range};
    cv::Mat hist;
    cv::calcHist(&lab, 1, channels, mask, hist, 3, histSize, ranges);
    const double sum = cv::sum(hist)[0];
    if (sum <= 0.0) return {};
    hist /= float(sum);
    return hist;
}

struct ColorSignature {
    cv::Mat hsHist;
    cv::Mat labHist;
    cv::Vec3d coreLab{0.0, 128.0, 128.0};
    double chromaticFrac = 0.0;
};

ColorSignature extractColorSignature(const cv::Mat &bgrPatch, const cv::Mat &fgMask = cv::Mat())
{
    ColorSignature sig;
    if (bgrPatch.empty()) return sig;

    cv::Mat hsv, lab;
    cv::cvtColor(bgrPatch, hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(bgrPatch, lab, cv::COLOR_BGR2Lab);

    cv::Mat validMask;
    if (!fgMask.empty() && fgMask.size() == bgrPatch.size() && cv::countNonZero(fgMask) >= 24) {
        validMask = fgMask;
    } else {
        validMask = cv::Mat(bgrPatch.rows, bgrPatch.cols, CV_8UC1, cv::Scalar(255));
    }

    cv::Mat chromMask(bgrPatch.rows, bgrPatch.cols, CV_8UC1, cv::Scalar(0));
    int validPixels = 0;
    int chromPixels = 0;
    cv::Vec3d chromLabSum(0.0, 0.0, 0.0);
    cv::Vec3d allLabSum(0.0, 0.0, 0.0);

    for (int r = 0; r < hsv.rows; ++r) {
        const auto *hRow = hsv.ptr<cv::Vec3b>(r);
        const auto *lRow = lab.ptr<cv::Vec3b>(r);
        const auto *vRow = validMask.ptr<uchar>(r);
        auto *cRow = chromMask.ptr<uchar>(r);
        for (int c = 0; c < hsv.cols; ++c) {
            if (!vRow[c]) continue;
            ++validPixels;
            allLabSum[0] += lRow[c][0];
            allLabSum[1] += lRow[c][1];
            allLabSum[2] += lRow[c][2];
            if (hRow[c][1] >= kSatFloor && hRow[c][2] >= kValFloor) {
                cRow[c] = 255;
                ++chromPixels;
                chromLabSum[0] += lRow[c][0];
                chromLabSum[1] += lRow[c][1];
                chromLabSum[2] += lRow[c][2];
            }
        }
    }

    if (validPixels <= 0) return sig;
    sig.chromaticFrac = double(chromPixels) / double(validPixels);
    sig.coreLab = (chromPixels >= 16) ? (chromLabSum / double(chromPixels))
                                      : (allLabSum / double(validPixels));
    sig.labHist = computeLabHist(bgrPatch, validMask);

    if (chromPixels >= 16) {
        const int channels[] = {0, 1};
        const int histSize[] = {24, 12};
        const float hRange[] = {0.0f, 180.0f};
        const float sRange[] = {0.0f, 256.0f};
        const float *ranges[] = {hRange, sRange};
        cv::calcHist(&hsv, 1, channels, chromMask, sig.hsHist, 2, histSize, ranges);
        const double sum = cv::sum(sig.hsHist)[0];
        if (sum > 0.0) sig.hsHist /= float(sum);
    }
    return sig;
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

cv::Mat buildChromaticBackProjection(const cv::Mat &bgr, const cv::Mat &hsHist)
{
    if (bgr.empty() || hsHist.empty()) return {};
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    const int channels[] = {0, 1};
    const float hRange[] = {0.0f, 180.0f};
    const float sRange[] = {0.0f, 256.0f};
    const float *ranges[] = {hRange, sRange};
    cv::Mat prob;
    cv::Mat scaledHist = hsHist * 255.0f;
    double maxVal = 0.0;
    cv::minMaxLoc(scaledHist, nullptr, &maxVal);
    if (maxVal > 1e-3) scaledHist *= float(255.0 / maxVal);
    cv::calcBackProject(&hsv, 1, channels, scaledHist, prob, ranges, 1.0);

    for (int r = 0; r < hsv.rows; ++r) {
        const auto *hRow = hsv.ptr<cv::Vec3b>(r);
        auto *pRow = prob.ptr<uchar>(r);
        for (int c = 0; c < hsv.cols; ++c) {
            if (hRow[c][1] < kSatFloor || hRow[c][2] < kValFloor) pRow[c] = 0;
        }
    }
    return prob;
}

cv::Mat smoothHsHistForCrossView(const cv::Mat &hsHist)
{
    if (hsHist.empty() || hsHist.rows != 24 || hsHist.cols != 12) return hsHist.clone();
    cv::Mat out = cv::Mat::zeros(hsHist.size(), CV_32F);
    static constexpr float kW[3][3] = {
        {0.015f, 0.15f, 0.015f},
        {0.070f, 0.50f, 0.070f},
        {0.015f, 0.15f, 0.015f}
    };
    for (int h = 0; h < 24; ++h) {
        for (int s = 0; s < 12; ++s) {
            float acc = 0.0f;
            for (int dh = -1; dh <= 1; ++dh) {
                const int nh = (h + dh + 24) % 24;
                for (int ds = -1; ds <= 1; ++ds) {
                    const int ns = std::clamp(s + ds, 0, 11);
                    acc += hsHist.at<float>(nh, ns) * kW[dh + 1][ds + 1];
                }
            }
            out.at<float>(h, s) = acc;
        }
    }
    const double sum = cv::sum(out)[0];
    if (sum > 0.0) out /= float(sum);
    return out;
}

// 依据完整外接框构造稳定检测结果；当框明显狭长（如立放/横躺瓶子）时自动赋予对应主轴角
Detection detectionFromBox(const cv::Rect &box, double refArea, double refBoxArea,
                           double fallbackAngle, bool fallbackAngleReliable, double score)
{
    Detection d;
    d.box = box;
    d.rotBox = cv::RotatedRect(
        cv::Point2f(box.x + box.width * 0.5f, box.y + box.height * 0.5f),
        cv::Size2f(float(box.width), float(box.height)), 0.0f);
    d.center = cv::Point2d(box.x + box.width * 0.5, box.y + box.height * 0.5);
    const double areaScale = double(std::max(1, box.area())) / std::max(1.0, refBoxArea);
    d.area = std::max(1.0, refArea * areaScale);
    d.aspectRatio = double(box.height) / std::max(1, box.width);
    if (d.aspectRatio >= 1.45) {
        d.angle = 90.0;
        d.angleReliable = true;
    } else if (d.aspectRatio <= 1.0 / 1.45) {
        d.angle = 0.0;
        d.angleReliable = true;
    } else {
        d.angle = fallbackAngle;
        d.angleReliable = fallbackAngleReliable;
    }
    d.score = std::clamp(score, 0.0, 1.0);
    d.valid = (box.width >= 8 && box.height >= 8);
    return d;
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

void AutoContourDetector::resetRecoveryCandidate()
{
    m_recoveryCandBox = cv::Rect();
    m_recoveryCandHits = 0;
}

void AutoContourDetector::resetTracking()
{
    m_hasPickedPoint = false;
    m_hasPendingRoi = false;
    m_pendingRoi = cv::Rect();
    m_roiMode = false;
    m_crossViewSearching = false;
    m_sourceFrameSize = cv::Size();
    m_referenceFromContour = false;
    m_yoloClassId = -1;
    m_yoloClassName.clear();
    m_yoloBoxAnchored = false;
    m_lastYoloAnchorMs = -1;
    m_lastRecoveryMs = -1;
    m_csrt.release();
    m_csrtActive = false;
    m_csrtScale = 1.0;
    m_lostFrames = 0;
    m_framesSinceCsrtReinit = 0;
    m_unreliableCount = 0;
    m_hasSmoothedBox = false;
    m_smoothedBox = cv::Rect2d();
    m_hasSmoothedAngle = false;
    m_smoothedAngle = 0.0;
    m_lastReliableDet = Detection();
    m_lastReliableContour.clear();
    resetRecoveryCandidate();
    m_refGrayTemplate.release();
    m_refOrbKeypoints.clear();
    m_refOrbDescriptors.release();
    m_refOrbPatchSize = cv::Size();
    m_refHsHist.release();
    m_crossHsHist.release();
    m_refLabHist.release();
    m_refCoreLab = cv::Vec3d(0.0, 128.0, 128.0);
    m_refChromaticFrac = 0.0;
    m_isChromatic = false;
    m_bgRingReliable = false;
    m_trackedScale = 1.0;
    m_kalmanReady = false;
    m_hasTrackedTarget = false;
    m_trackedArea = 0.0;
    m_reference = Detection();
    m_referenceContour.clear();
}

void AutoContourDetector::buildOrbReference(const cv::Mat &bgr, const cv::Rect &box)
{
    m_refOrbKeypoints.clear();
    m_refOrbDescriptors.release();
    m_refOrbPatchSize = cv::Size();
    const cv::Rect clamped = clampRect(box, bgr.size());
    if (bgr.empty() || clamped.width < 16 || clamped.height < 16) return;
    try {
        cv::Mat gray;
        cv::cvtColor(bgr(clamped), gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0);
        auto orb = cv::ORB::create(500);
        orb->detectAndCompute(gray, cv::noArray(), m_refOrbKeypoints, m_refOrbDescriptors);
        m_refOrbPatchSize = clamped.size();
    } catch (const cv::Exception &) {
        m_refOrbKeypoints.clear();
        m_refOrbDescriptors.release();
    }
}

TargetSignature AutoContourDetector::exportSignature(const cv::Mat &bgr) const
{
    TargetSignature sig;
    if (!m_reference.valid || bgr.empty()) return sig;

    sig.refBoxSize = m_reference.box.size();
    sig.refArea = std::max(1.0, m_reference.area);
    sig.sourceFrameSize = bgr.size();
    sig.aspectRatio = m_reference.aspectRatio;
    sig.yoloClassId = m_yoloClassId;
    sig.yoloClassName = m_yoloClassName;
    sig.orbKeypoints = m_refOrbKeypoints;
    sig.orbDescriptors = m_refOrbDescriptors.clone();
    sig.orbPatchSize = m_refOrbPatchSize;

    if (m_roiMode && !m_refLabHist.empty()) {
        sig.isChromatic = m_isChromatic;
        sig.hsHist = m_refHsHist.clone();
        sig.labHist = m_refLabHist.clone();
        sig.coreLab = m_refCoreLab;
        sig.chromaticFrac = m_refChromaticFrac;
        sig.grayTemplate = m_refGrayTemplate.clone();
        sig.valid = true;
        return sig;
    }

    const cv::Rect clamped = clampRect(m_reference.box, bgr.size());
    if (clamped.width < 8 || clamped.height < 8) return sig;
    cv::Mat refMask;
    if (!m_referenceContour.empty()) {
        refMask = cv::Mat::zeros(clamped.height, clamped.width, CV_8UC1);
        std::vector<std::vector<cv::Point>> shifted(1);
        for (const auto &pt : m_referenceContour)
            shifted[0].emplace_back(pt.x - clamped.x, pt.y - clamped.y);
        cv::drawContours(refMask, shifted, 0, cv::Scalar(255), cv::FILLED);
    }
    const ColorSignature cs = refMask.empty()
        ? extractColorSignature(bgr(innerCoreRect(clamped, bgr.size())))
        : extractColorSignature(bgr(clamped), refMask);
    sig.hsHist = cs.hsHist.clone();
    sig.labHist = cs.labHist.clone();
    sig.coreLab = cs.coreLab;
    sig.chromaticFrac = cs.chromaticFrac;
    sig.isChromatic = (cs.chromaticFrac >= 0.22 && !cs.hsHist.empty());
    cv::cvtColor(bgr(clamped), sig.grayTemplate, cv::COLOR_BGR2GRAY);
    sig.valid = !sig.labHist.empty();
    return sig;
}

void AutoContourDetector::adoptSignature(const TargetSignature &sig)
{
    resetTracking();
    if (!sig.valid) return;

    m_roiMode = true;
    m_crossViewSearching = true;
    m_sourceFrameSize = sig.sourceFrameSize;
    m_isChromatic = sig.isChromatic;
    m_yoloClassId = sig.yoloClassId;
    m_yoloClassName = sig.yoloClassName;
    m_yoloBoxAnchored = (m_yoloClassId >= 0);
    m_refHsHist = sig.hsHist.clone();
    m_crossHsHist = smoothHsHistForCrossView(m_refHsHist);
    m_refLabHist = sig.labHist.clone();
    m_refCoreLab = sig.coreLab;
    m_refChromaticFrac = sig.chromaticFrac;
    m_refGrayTemplate = sig.grayTemplate.clone();
    m_refOrbKeypoints = sig.orbKeypoints;
    m_refOrbDescriptors = sig.orbDescriptors.clone();
    m_refOrbPatchSize = sig.orbPatchSize;

    m_reference = Detection();
    m_reference.box = cv::Rect(0, 0, std::max(12, sig.refBoxSize.width), std::max(12, sig.refBoxSize.height));
    m_reference.area = std::max(100.0, sig.refArea);
    m_reference.aspectRatio = sig.aspectRatio > 0.0 ? sig.aspectRatio : 1.0;
    m_reference.valid = true;
    m_lostFrames = 0;
}

void AutoContourDetector::initKalmanAt(const cv::Point2d &center)
{
    m_kalman.init(4, 2, 0, CV_32F);
    m_kalman.transitionMatrix = (cv::Mat_<float>(4, 4) <<
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);
    cv::setIdentity(m_kalman.measurementMatrix);
    cv::setIdentity(m_kalman.processNoiseCov, cv::Scalar::all(1e-2));
    cv::setIdentity(m_kalman.measurementNoiseCov, cv::Scalar::all(2.0));
    cv::setIdentity(m_kalman.errorCovPost, cv::Scalar::all(1.0));
    m_kalman.statePost = (cv::Mat_<float>(4, 1) << float(center.x), float(center.y), 0.0f, 0.0f);
    m_kalmanReady = true;
}

void AutoContourDetector::initCsrtTracker(const cv::Mat &bgr, const cv::Rect &fullBox)
{
    m_csrt.release();
    m_csrtActive = false;
    const cv::Rect clamped = clampRect(fullBox, bgr.size());
    if (clamped.width < 10 || clamped.height < 10) return;

    m_csrtScale = (bgr.cols >= 480) ? 0.5 : 1.0;
    cv::Mat smallBgr;
    if (m_csrtScale < 1.0) {
        cv::resize(bgr, smallBgr, cv::Size(), m_csrtScale, m_csrtScale, cv::INTER_AREA);
    } else {
        smallBgr = bgr;
    }

    const cv::Rect smallBox = clampRect(
        cv::Rect(int(std::lround(clamped.x * m_csrtScale)),
                 int(std::lround(clamped.y * m_csrtScale)),
                 std::max(8, int(std::lround(clamped.width * m_csrtScale))),
                 std::max(8, int(std::lround(clamped.height * m_csrtScale)))),
        smallBgr.size());
    if (smallBox.width < 8 || smallBox.height < 8) return;

    cv::TrackerCSRT::Params params;
    params.use_hog = true;
    params.use_color_names = true;
    params.use_rgb = true;
    params.use_gray = true;
    params.use_channel_weights = true;
    params.use_segmentation = true;
    params.admm_iterations = 3;
    params.template_size = 160;
    params.filter_lr = 0.012f;     // 更保守的在线学习率，显著减少目标静止/反光时的尺度漂移
    params.histogram_lr = 0.015f;
    params.psr_threshold = 0.035f;

    try {
        m_csrt = cv::TrackerCSRT::create(params);
        m_csrt->init(smallBgr, smallBox);
        m_csrtActive = true;
        m_framesSinceCsrtReinit = 0;
        m_unreliableCount = 0;
    } catch (const cv::Exception &) {
        m_csrt.release();
        m_csrtActive = false;
    }
}

bool AutoContourDetector::confirmRecoveryCandidate(const cv::Rect &box, int requiredFrames)
{
    if (requiredFrames <= 1) {
        resetRecoveryCandidate();
        return true;
    }
    if (box.width < 8 || box.height < 8) {
        resetRecoveryCandidate();
        return false;
    }
    bool same = false;
    if (m_recoveryCandBox.width > 0 && m_recoveryCandBox.height > 0) {
        const cv::Point2d curC(box.x + box.width * 0.5, box.y + box.height * 0.5);
        const cv::Point2d prevC(m_recoveryCandBox.x + m_recoveryCandBox.width * 0.5,
                                m_recoveryCandBox.y + m_recoveryCandBox.height * 0.5);
        const double dist = cv::norm(curC - prevC);
        const double maxDist = std::max(18.0, std::max(box.width, box.height) * 0.45);
        const double curArea = std::max(1.0, double(box.area()));
        const double prevArea = std::max(1.0, double(m_recoveryCandBox.area()));
        const double areaSim = std::min(curArea, prevArea) / std::max(curArea, prevArea);
        same = (dist <= maxDist && areaSim >= 0.55);
    }
    if (same) {
        ++m_recoveryCandHits;
    } else {
        m_recoveryCandHits = 1;
    }
    m_recoveryCandBox = box;
    if (m_recoveryCandHits >= requiredFrames) {
        resetRecoveryCandidate();
        return true;
    }
    return false;
}

// 借鉴 untitled7::smoothYoloTargetRect：中心位置死区 + 尺寸死区 + 非对称指数平滑，
// 彻底消除静止目标的“框微抖误报移动”与“框呼吸误报变大/变小”。
cv::Rect AutoContourDetector::smoothOutputBox(const cv::Rect &detectedBox, const cv::Size &frameSize, bool snap)
{
    const cv::Rect clamped = clampRect(detectedBox, frameSize);
    if (clamped.width < 8 || clamped.height < 8) return clamped;
    if (!m_boxSmoothingEnabled || snap || !m_hasSmoothedBox
        || m_smoothedBox.width <= 1.0 || m_smoothedBox.height <= 1.0) {
        m_smoothedBox = cv::Rect2d(clamped.x, clamped.y, clamped.width, clamped.height);
        m_hasSmoothedBox = true;
        return clamped;
    }

    const double prevCx = m_smoothedBox.x + m_smoothedBox.width * 0.5;
    const double prevCy = m_smoothedBox.y + m_smoothedBox.height * 0.5;
    const double detCx = clamped.x + clamped.width * 0.5;
    const double detCy = clamped.y + clamped.height * 0.5;
    const double movement = std::hypot(detCx - prevCx, detCy - prevCy);
    const double scale = std::max(20.0, std::max(m_smoothedBox.width, m_smoothedBox.height));
    const double movementRatio = movement / scale;

    const double prevAspect = m_smoothedBox.height / std::max(1.0, m_smoothedBox.width);
    const double detAspect = double(clamped.height) / std::max(1, clamped.width);
    const bool orientationFlipped =
        (prevAspect >= 1.30 && detAspect <= 0.85) || (prevAspect <= 0.77 && detAspect >= 1.18);

    // 中心位置死区：静止目标 1~5px 的 CSRT/YOLO 微抖直接锁死为 0 位移，不让框乱晃
    const double posDeadZone = std::max(3.5, scale * 0.025);
    double centerAlpha = 0.0;
    if (orientationFlipped || movementRatio > 0.25) centerAlpha = 1.0;
    else if (movementRatio > 0.12) centerAlpha = 0.85;
    else if (movement > posDeadZone * 2.0) centerAlpha = 0.58;
    else if (movement > posDeadZone) centerAlpha = 0.28;
    else centerAlpha = 0.0;

    auto smoothDim = [&](double prevVal, double detVal) {
        if (orientationFlipped) return prevVal + (detVal - prevVal) * 0.85;
        const double diff = detVal - prevVal;
        const double deadZone = std::max(kSizeDeadZonePixels, scale * kSizeDeadZoneRatio);
        if (std::abs(diff) <= deadZone) return prevVal;
        const bool expanding = diff > 0.0;
        const double ratio = std::abs(diff) / std::max(1.0, prevVal);
        double alpha = expanding ? 0.36 : 0.14;
        if (ratio > 0.24) alpha = expanding ? 0.58 : 0.28;
        return prevVal + diff * alpha;
    };

    const double newW = smoothDim(m_smoothedBox.width, double(clamped.width));
    const double newH = smoothDim(m_smoothedBox.height, double(clamped.height));
    const double newCx = prevCx + (detCx - prevCx) * centerAlpha;
    const double newCy = prevCy + (detCy - prevCy) * centerAlpha;
    m_smoothedBox = cv::Rect2d(newCx - newW * 0.5, newCy - newH * 0.5, newW, newH);

    return clampRect(cv::Rect(int(std::lround(m_smoothedBox.x)),
                              int(std::lround(m_smoothedBox.y)),
                              std::max(8, int(std::lround(m_smoothedBox.width))),
                              std::max(8, int(std::lround(m_smoothedBox.height)))),
                     frameSize);
}

double AutoContourDetector::smoothOutputAngle(double angleDeg, bool snap)
{
    if (!m_boxSmoothingEnabled || snap || !m_hasSmoothedAngle) {
        m_smoothedAngle = angleDeg;
        m_hasSmoothedAngle = true;
        return angleDeg;
    }
    // 180° 周期最小有符号角差 [-90, +90]
    double diff = std::fmod(angleDeg - m_smoothedAngle + 270.0, 180.0) - 90.0;
    if (std::abs(diff) <= 2.5) return m_smoothedAngle;
    const double alpha = (std::abs(diff) >= 20.0) ? 0.65 : 0.30;
    m_smoothedAngle += diff * alpha;
    // 归一化到 [-90, +90]
    m_smoothedAngle = std::fmod(m_smoothedAngle + 270.0, 180.0) - 90.0;
    return m_smoothedAngle;
}

AutoContourDetector::IdentityScore AutoContourDetector::evaluateIdentity(
    const cv::Mat &bgr, const cv::Rect &box, const cv::Mat *fgMask,
    bool strictForRedetect, bool crossView) const
{
    IdentityScore res;
    const cv::Rect clamped = clampRect(box, bgr.size());
    if (clamped.width < 8 || clamped.height < 8) return res;

    cv::Mat patch;
    cv::Mat maskToUse;
    if (fgMask && !fgMask->empty() && fgMask->size() == clamped.size() && cv::countNonZero(*fgMask) >= 24) {
        patch = bgr(clamped);
        maskToUse = *fgMask;
    } else {
        const cv::Rect core = innerCoreRect(clamped, bgr.size());
        patch = bgr(core);
    }

    const ColorSignature curr = extractColorSignature(patch, maskToUse);
    res.chromaticFrac = curr.chromaticFrac;
    res.abDist = std::hypot(curr.coreLab[1] - m_refCoreLab[1], curr.coreLab[2] - m_refCoreLab[2]);
    if (!m_refLabHist.empty() && !curr.labHist.empty()) {
        res.labSim = std::max(0.0, 1.0 - cv::compareHist(m_refLabHist, curr.labHist, cv::HISTCMP_BHATTACHARYYA));
    }
    if (m_isChromatic) {
        const cv::Mat &refHs = (crossView && !m_crossHsHist.empty()) ? m_crossHsHist : m_refHsHist;
        if (!refHs.empty() && !curr.hsHist.empty()) {
            res.hsSim = std::max(0.0, 1.0 - cv::compareHist(refHs, curr.hsHist, cv::HISTCMP_BHATTACHARYYA));
        }
        const double minChromFrac = std::max(0.08, m_refChromaticFrac * (crossView ? 0.30 : (strictForRedetect ? 0.42 : 0.28)));
        const double minHsSim = crossView ? 0.42 : (strictForRedetect ? 0.48 : 0.36);
        const double maxAbDist = crossView ? 42.0 : (strictForRedetect ? 32.0 : 42.0);
        if (curr.chromaticFrac < minChromFrac || res.hsSim < minHsSim || res.abDist > maxAbDist) {
            return res;
        }
        res.valid = true;
        res.score = 0.55 * res.hsSim + 0.30 * std::max(0.0, 1.0 - res.abDist / 45.0) + 0.15 * res.labSim;
        return res;
    }

    // 非彩色（黑白/灰/金属质感）目标：使用 Lab 分布 + 防串彩色物体 + 灰度相关性校验（兼容 90° 倾倒）
    const double maxChromForAchromatic = strictForRedetect ? 0.38 : 0.48;
    const double minLabSim = crossView ? 0.40 : (strictForRedetect ? 0.52 : 0.42);
    if (curr.chromaticFrac > maxChromForAchromatic || res.labSim < minLabSim) {
        return res;
    }
    double grayCorr = 0.5;
    if (!m_refGrayTemplate.empty()) {
        cv::Mat gPatch;
        cv::cvtColor(bgr(clamped), gPatch, cv::COLOR_BGR2GRAY);
        auto evalTemplate = [&](const cv::Mat &tmpl) {
            cv::Mat gResized, corr;
            cv::resize(gPatch, gResized, tmpl.size(), 0, 0, cv::INTER_AREA);
            cv::matchTemplate(gResized, tmpl, corr, cv::TM_CCOEFF_NORMED);
            return double(corr.at<float>(0, 0));
        };
        grayCorr = evalTemplate(m_refGrayTemplate);
        const double minCorr = crossView ? 0.38 : (strictForRedetect ? 0.45 : 0.30);
        if (grayCorr < minCorr) {
            cv::Mat rotCw, rotCcw;
            cv::rotate(m_refGrayTemplate, rotCw, cv::ROTATE_90_CLOCKWISE);
            cv::rotate(m_refGrayTemplate, rotCcw, cv::ROTATE_90_COUNTERCLOCKWISE);
            grayCorr = std::max({grayCorr, evalTemplate(rotCw), evalTemplate(rotCcw)});
        }
        if (grayCorr < minCorr) return res;
    }
    res.valid = true;
    res.score = 0.55 * res.labSim + 0.45 * std::clamp(grayCorr, 0.0, 1.0);
    return res;
}

bool AutoContourDetector::refineContourInBox(const cv::Mat &bgr, const cv::Rect &box,
                                             Detection &outDet, std::vector<cv::Point> &outContour) const
{
    const int refMaxDim = std::max(m_reference.box.width, m_reference.box.height);
    const int boxMaxDim = std::max(box.width, box.height);
    const int desiredSide = int(std::lround(
        std::max(double(boxMaxDim), double(refMaxDim) * std::clamp(m_trackedScale, 0.55, 1.8)) * 1.18));
    const int padX = std::clamp(std::max((desiredSide - box.width) / 2 + 12, box.width / 4), 12, 96);
    const int padY = std::clamp(std::max((desiredSide - box.height) / 2 + 12, box.height / 4), 12, 96);
    const cv::Rect win = clampRect(
        cv::Rect(box.x - padX, box.y - padY, box.width + 2 * padX, box.height + 2 * padY), bgr.size());
    if (win.width < 10 || win.height < 10) return false;

    const int minSegArea = std::max(36, int(m_reference.area * 0.14));
    // 1. 优先对彩色目标用 HSV 反投影掩膜提取轮廓（支持带中间白色标签的瓶身直立/倾倒合并）
    if (m_isChromatic && !m_refHsHist.empty()) {
        const cv::Mat patch = bgr(win);
        cv::Mat prob = buildChromaticBackProjection(patch, m_refHsHist);
        if (!prob.empty()) {
            cv::Mat mask;
            cv::threshold(prob, mask, 28, 255, cv::THRESH_BINARY);
            static const cv::Mat kOpen = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            static const cv::Mat kClose = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kOpen);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kClose);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            const cv::Rect localBox(box.x - win.x, box.y - win.y, box.width, box.height);
            const cv::Point2d localCenter(localBox.x + localBox.width * 0.5, localBox.y + localBox.height * 0.5);
            const double pieceMinArea = std::max(24.0, double(minSegArea) * 0.20);

            struct LocalPiece {
                size_t idx = 0;
                cv::Rect cb;
                double area = 0.0;
                bool isSeed = false;
            };
            std::vector<LocalPiece> pieces;
            for (size_t i = 0; i < contours.size(); ++i) {
                const double area = cv::contourArea(contours[i]);
                if (area < pieceMinArea) continue;
                const cv::Rect cb = cv::boundingRect(contours[i]);
                const cv::Rect inter = cb & localBox;
                const bool overlapsBox = (inter.area() > 0)
                    && (double(inter.area()) / std::max(1.0, double(cb.area())) >= 0.30
                        || double(inter.area()) >= pieceMinArea * 0.60
                        || cb.contains(cv::Point(int(std::lround(localCenter.x)), int(std::lround(localCenter.y)))));
                pieces.push_back({i, cb, area, overlapsBox});
            }

            const double maxGap = std::max(10.0, double(desiredSide) * 0.42);
            const double maxUnionSpan = double(desiredSide) * 1.55 + 14.0;
            std::vector<bool> selected(pieces.size(), false);
            cv::Rect unionBox;
            bool changed = false;
            for (size_t i = 0; i < pieces.size(); ++i) {
                if (pieces[i].isSeed) {
                    selected[i] = true;
                    unionBox = unionBox.empty() ? pieces[i].cb : (unionBox | pieces[i].cb);
                    changed = true;
                }
            }
            while (changed) {
                changed = false;
                for (size_t i = 0; i < pieces.size(); ++i) {
                    if (selected[i]) continue;
                    const cv::Rect &cand = pieces[i].cb;
                    const cv::Rect merged = unionBox | cand;
                    if (std::max(merged.width, merged.height) > maxUnionSpan) continue;
                    bool nearSelected = false;
                    for (size_t j = 0; j < pieces.size(); ++j) {
                        if (!selected[j]) continue;
                        const cv::Rect &a = pieces[j].cb;
                        const int gapX = std::max(0, std::max(a.x, cand.x) - std::min(a.x + a.width, cand.x + cand.width));
                        const int gapY = std::max(0, std::max(a.y, cand.y) - std::min(a.y + a.height, cand.y + cand.height));
                        if (gapX <= maxGap && gapY <= maxGap && std::hypot(gapX, gapY) <= maxGap) {
                            nearSelected = true;
                            break;
                        }
                    }
                    if (nearSelected) {
                        selected[i] = true;
                        unionBox = merged;
                        changed = true;
                    }
                }
            }

            std::vector<size_t> matchedIdx;
            double totalPieceArea = 0.0;
            for (size_t i = 0; i < pieces.size(); ++i) {
                if (!selected[i]) continue;
                matchedIdx.push_back(pieces[i].idx);
                totalPieceArea += pieces[i].area;
            }

            if (!matchedIdx.empty() && totalPieceArea >= double(minSegArea) * 0.65) {
                std::vector<cv::Point> cFull;
                if (matchedIdx.size() == 1) {
                    cFull = contours[matchedIdx[0]];
                } else {
                    std::vector<cv::Point> allPts;
                    for (size_t idx : matchedIdx)
                        allPts.insert(allPts.end(), contours[idx].begin(), contours[idx].end());
                    cv::convexHull(allPts, cFull);
                }
                for (auto &pt : cFull) { pt.x += win.x; pt.y += win.y; }
                const Detection d = detectionFromContour(cFull);
                const double boxArea = std::max(1.0, double(box.area()));
                const int dMax = std::max(d.box.width, d.box.height);
                const int dMin = std::min(d.box.width, d.box.height);
                const int bMax = std::max(box.width, box.height);
                const int bMin = std::min(box.width, box.height);
                const bool clippedLabeledTarget = (m_refChromaticFrac < 0.88)
                    && (dMax < bMax * 0.68 || dMin < bMin * 0.58);
                if (d.valid && !clippedLabeledTarget
                    && d.area >= boxArea * 0.22 && d.area <= boxArea * 3.0) {
                    outDet = d;
                    outContour = std::move(cFull);
                    return true;
                }
            }
        }
    }

    // 2. 退回到局部 Lab 背景对比轮廓细化（覆盖黑白/灰度目标或高对比桌面）
    if (m_referenceFromContour && m_bgRingReliable) {
        Detection d;
        std::vector<cv::Point> c;
        if (segmentTargetInBox(bgr, win, m_bgLabMean, m_bgLabSpread, true, minSegArea, d, c)) {
            const double boxArea = std::max(1.0, double(box.area()));
            if (d.area >= boxArea * 0.22 && d.area <= boxArea * 3.0) {
                outDet = d;
                outContour = std::move(c);
                return true;
            }
        }
    }
    return false;
}

bool AutoContourDetector::redetectByYolo(const cv::Mat &bgr, Detection &outDet,
                                         std::vector<cv::Point> &outContour, bool crossView)
{
    if (!m_yoloEnabled || m_yoloClassId < 0) return false;
    auto &yolo = YoloDetector::instance();
    if (!yolo.ensureLoaded()) return false;

    const auto boxes = yolo.detect(bgr, m_yoloClassId, /*tryRotated90IfEmpty=*/true);
    if (boxes.empty()) return false;

    const double frameDiag = std::max(1.0, std::hypot(bgr.cols, bgr.rows));
    const double areaScale = (crossView && m_sourceFrameSize.width > 0 && m_sourceFrameSize.height > 0)
        ? (double(bgr.cols * bgr.rows) / double(m_sourceFrameSize.width * m_sourceFrameSize.height))
        : 1.0;
    const double lenScale = std::sqrt(areaScale);
    const cv::Size expectedSize(std::max(12, int(std::lround(m_reference.box.width * lenScale))),
                                std::max(12, int(std::lround(m_reference.box.height * lenScale))));

    double bestScore = -1.0;
    YoloBox bestMatch;
    double bestIdScore = 0.0;

    for (const auto &yb : boxes) {
        if (yb.classId != m_yoloClassId) continue;
        if (yb.box.width < 12 || yb.box.height < 12) continue;
        const IdentityScore id = evaluateIdentity(bgr, yb.box, nullptr, true, crossView);
        if (!id.valid) continue;

        const double shapeSim = rotationInvariantShapeSimilarity(yb.box.size(), expectedSize);
        if (shapeSim < (crossView ? 0.30 : 0.25)) continue;

        const cv::Point2d cc(yb.box.x + yb.box.width * 0.5, yb.box.y + yb.box.height * 0.5);
        const double proxTerm = crossView
            ? 1.0
            : (1.0 - 0.16 * std::min(1.0, cv::norm(cc - m_trackedCenter) / frameDiag));
        const double score = (0.62 * id.score + 0.20 * shapeSim + 0.18 * std::clamp(double(yb.confidence), 0.0, 1.0)) * proxTerm;
        if (score > bestScore) {
            bestScore = score;
            bestMatch = yb;
            bestIdScore = id.score;
        }
    }

    if (bestScore <= 0.0 || bestMatch.box.width < 12 || bestMatch.box.height < 12)
        return false;

    const double refBoxArea = std::max(1.0, double(m_reference.box.area()) * areaScale);
    const double refArea = m_yoloBoxAnchored ? double(bestMatch.box.area()) : (m_reference.area * areaScale);
    outDet = detectionFromBox(bestMatch.box, crossView ? double(bestMatch.box.area()) : refArea,
                              crossView ? double(bestMatch.box.area()) : refBoxArea,
                              m_reference.angle, m_reference.angleReliable, bestIdScore);
    outContour.clear();
    // 尝试提取内部轮廓仅用于精细倾角，绝不缩小完整 YOLO 外接框
    Detection refined;
    std::vector<cv::Point> c;
    if (refineContourInBox(bgr, bestMatch.box, refined, c)) {
        const int rMax = std::max(refined.box.width, refined.box.height);
        const int yMax = std::max(bestMatch.box.width, bestMatch.box.height);
        if (refined.angleReliable && rMax >= yMax * 0.65) {
            outDet.angle = refined.angle;
            outDet.angleReliable = true;
            outContour = std::move(c);
        }
    }
    return true;
}

bool AutoContourDetector::redetectByOrb(const cv::Mat &bgr, Detection &outDet,
                                        std::vector<cv::Point> &outContour, bool crossView) const
{
    if (bgr.empty() || m_refOrbDescriptors.empty()
        || m_refOrbKeypoints.size() < size_t(kOrbMinGoodMatches)
        || m_refOrbPatchSize.width < 16 || m_refOrbPatchSize.height < 16) {
        return false;
    }

    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0);

    std::vector<cv::KeyPoint> sceneKps;
    cv::Mat sceneDesc;
    try {
        auto orb = cv::ORB::create(1200);
        orb->detectAndCompute(gray, cv::noArray(), sceneKps, sceneDesc);
    } catch (const cv::Exception &) {
        return false;
    }
    if (sceneDesc.empty() || sceneKps.size() < size_t(kOrbMinGoodMatches)) return false;

    std::vector<std::vector<cv::DMatch>> knn;
    try {
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        matcher.knnMatch(m_refOrbDescriptors, sceneDesc, knn, 2);
    } catch (const cv::Exception &) {
        return false;
    }

    std::vector<cv::DMatch> good;
    for (const auto &pair : knn) {
        if (pair.size() < 2) continue;
        if (pair[0].distance < 0.76f * pair[1].distance && pair[0].distance < 68.0f) {
            good.push_back(pair[0]);
        }
    }
    if (good.size() < size_t(kOrbMinGoodMatches)) return false;

    std::vector<double> dxs, dys;
    dxs.reserve(good.size());
    dys.reserve(good.size());
    for (const auto &m : good) {
        const cv::Point2f &sp = m_refOrbKeypoints[m.queryIdx].pt;
        const cv::Point2f &dp = sceneKps[m.trainIdx].pt;
        dxs.push_back(dp.x - sp.x);
        dys.push_back(dp.y - sp.y);
    }
    const double medX = medianDouble(dxs);
    const double medY = medianDouble(dys);

    int inliers = 0;
    for (const auto &m : good) {
        const cv::Point2f &sp = m_refOrbKeypoints[m.queryIdx].pt;
        const cv::Point2f &dp = sceneKps[m.trainIdx].pt;
        if (std::abs((dp.x - sp.x) - medX) <= std::max(10.0, m_refOrbPatchSize.width * 0.28)
            && std::abs((dp.y - sp.y) - medY) <= std::max(10.0, m_refOrbPatchSize.height * 0.28)) {
            ++inliers;
        }
    }
    if (inliers < kOrbMinInliers) return false;

    const cv::Rect candBox = clampRect(
        cv::Rect(int(std::lround(medX)), int(std::lround(medY)),
                 m_refOrbPatchSize.width, m_refOrbPatchSize.height),
        bgr.size());
    if (candBox.width < 12 || candBox.height < 12) return false;

    const IdentityScore id = evaluateIdentity(bgr, candBox, nullptr, true, crossView);
    if (!id.valid) return false;

    outDet = detectionFromBox(candBox, m_reference.area, std::max(1.0, double(m_reference.box.area())),
                              m_reference.angle, m_reference.angleReliable, id.score);
    outContour.clear();
    return true;
}

bool AutoContourDetector::redetectTarget(const cv::Mat &bgr, Detection &outDet,
                                         std::vector<cv::Point> &outContour, bool crossView)
{
    // 0. 借鉴 untitled7：若目标已被 YOLO26n 判定为明确语义类别（如瓶子/杯子/手机），
    //    跨视角联动与丢失重捕严格只接受“同类 YOLO 检测框 + 初始颜色外观校验”，
    //    绝不回退到纯颜色反投影去误锁柜子边角、小黄标签或背景纸箱！
    if (m_yoloEnabled && m_yoloClassId >= 0 && YoloDetector::instance().isReady()) {
        return redetectByYolo(bgr, outDet, outContour, crossView);
    }

    const double ds = (bgr.cols >= 480) ? 0.5 : 1.0;
    cv::Mat smallBgr;
    if (ds < 1.0) cv::resize(bgr, smallBgr, cv::Size(), ds, ds, cv::INTER_AREA);
    else smallBgr = bgr;

    const double frameDiag = std::max(1.0, std::hypot(bgr.cols, bgr.rows));
    const double areaScale = (crossView && m_sourceFrameSize.width > 0 && m_sourceFrameSize.height > 0)
        ? (double(bgr.cols * bgr.rows) / double(m_sourceFrameSize.width * m_sourceFrameSize.height))
        : 1.0;
    const double lenScale = std::sqrt(areaScale);
    const double expectedArea = std::max(100.0, m_reference.area * areaScale);
    const cv::Size expectedSize(std::max(12, int(std::lround(m_reference.box.width * lenScale))),
                                std::max(12, int(std::lround(m_reference.box.height * lenScale))));

    // 路径 A：彩色目标 —— HSV 颜色反投影 + 旋转无关形状/尺寸门控（防止跨视角误锁微小同色噪点/贴纸）
    const cv::Mat &searchHs = (crossView && !m_crossHsHist.empty()) ? m_crossHsHist : m_refHsHist;
    if (m_isChromatic && !searchHs.empty()) {
        cv::Mat prob = buildChromaticBackProjection(smallBgr, searchHs);
        if (!prob.empty()) {
            cv::Mat mask;
            cv::threshold(prob, mask, crossView ? 28 : 32, 255, cv::THRESH_BINARY);
            static const cv::Mat kOpen = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            static const cv::Mat kClose = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kOpen);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kClose);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // 跨视角收紧最小面积底限，阻止 25x25 级微小黄色贴纸假冒整只瓶子
            const double minAreaSmall = std::max(crossView ? 36.0 : 20.0,
                                                 expectedArea * ds * ds * (crossView ? 0.30 : 0.16));
            const double maxAreaSmall = std::min(double(smallBgr.cols * smallBgr.rows) * 0.65,
                                                 expectedArea * ds * ds * (crossView ? 4.5 : 4.2));
            const double pieceAreaSmall = std::max(10.0, minAreaSmall * 0.22);
            const double refWSmall = std::max(8.0, m_reference.box.width * lenScale * ds);
            const double refHSmall = std::max(8.0, m_reference.box.height * lenScale * ds);
            const double refMaxSmall = std::max(refWSmall, refHSmall);

            struct BlobPiece {
                cv::Rect box;
                double area = 0.0;
                std::vector<cv::Point> pts;
            };
            std::vector<BlobPiece> pieces;
            for (const auto &c : contours) {
                const double a = cv::contourArea(c);
                if (a < pieceAreaSmall || a > maxAreaSmall) continue;
                pieces.push_back({cv::boundingRect(c), a, c});
            }

            std::vector<int> parent(pieces.size());
            for (size_t i = 0; i < pieces.size(); ++i) parent[i] = int(i);
            auto findRoot = [&](int x) {
                while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
                return x;
            };

            const double maxGap = std::max(6.0, refMaxSmall * (crossView ? 0.48 : 0.40));
            const double maxUnionSpan = refMaxSmall * (crossView ? 2.20 : 1.75) + 8.0;

            for (size_t i = 0; i < pieces.size(); ++i) {
                for (size_t j = i + 1; j < pieces.size(); ++j) {
                    const cv::Rect &a = pieces[i].box;
                    const cv::Rect &b = pieces[j].box;
                    const int gapX = std::max(0, std::max(a.x, b.x) - std::min(a.x + a.width, b.x + b.width));
                    const int gapY = std::max(0, std::max(a.y, b.y) - std::min(a.y + a.height, b.y + b.height));
                    if (gapX > maxGap || gapY > maxGap || std::hypot(gapX, gapY) > maxGap) continue;
                    const cv::Rect u = a | b;
                    if (std::max(u.width, u.height) > maxUnionSpan) continue;
                    const int ra = findRoot(int(i));
                    const int rb = findRoot(int(j));
                    if (ra != rb) parent[rb] = ra;
                }
            }

            std::vector<std::vector<size_t>> groups(pieces.size());
            for (size_t i = 0; i < pieces.size(); ++i) groups[size_t(findRoot(int(i)))].push_back(i);

            double bestScore = -1.0;
            Detection bestDet;
            std::vector<cv::Point> bestContour;

            for (const auto &grp : groups) {
                if (grp.empty()) continue;
                std::vector<cv::Point> cSmall;
                if (grp.size() == 1) {
                    if (pieces[grp[0]].area < minAreaSmall) continue;
                    cSmall = pieces[grp[0]].pts;
                } else {
                    std::vector<cv::Point> mergedPts;
                    double sumPieceArea = 0.0;
                    cv::Rect unionBox;
                    for (size_t idx : grp) {
                        sumPieceArea += pieces[idx].area;
                        unionBox = unionBox.empty() ? pieces[idx].box : (unionBox | pieces[idx].box);
                        mergedPts.insert(mergedPts.end(), pieces[idx].pts.begin(), pieces[idx].pts.end());
                    }
                    if (std::max(unionBox.width, unionBox.height) > maxUnionSpan) continue;
                    cv::convexHull(mergedPts, cSmall);
                    const double hullArea = cv::contourArea(cSmall);
                    if (hullArea < minAreaSmall || hullArea > maxAreaSmall || sumPieceArea < minAreaSmall * 0.50) continue;
                }

                std::vector<cv::Point> cFull;
                cFull.reserve(cSmall.size());
                for (const auto &pt : cSmall) {
                    cFull.emplace_back(std::clamp(int(std::lround(pt.x / ds)), 0, bgr.cols - 1),
                                       std::clamp(int(std::lround(pt.y / ds)), 0, bgr.rows - 1));
                }
                Detection cand = detectionFromContour(cFull);
                if (!cand.valid) continue;

                // 旋转无关形状相似度门控：拒绝尺寸/长宽比与目标悬殊的微小黄色贴纸或杂色块
                const double shapeSim = rotationInvariantShapeSimilarity(cand.box.size(), expectedSize);
                if (shapeSim < (crossView ? 0.44 : 0.34)) continue;

                cv::Mat localMask = cv::Mat::zeros(cand.box.height, cand.box.width, CV_8UC1);
                std::vector<std::vector<cv::Point>> shifted(1);
                shifted[0].reserve(cFull.size());
                for (const auto &pt : cFull) shifted[0].emplace_back(pt.x - cand.box.x, pt.y - cand.box.y);
                cv::drawContours(localMask, shifted, 0, cv::Scalar(255), cv::FILLED);

                const IdentityScore id = evaluateIdentity(bgr, cand.box, &localMask, true, crossView);
                if (!id.valid) continue;

                const double areaRatio = cand.area / expectedArea;
                const double areaTerm = std::exp((crossView ? -0.35 : -0.7) * std::abs(std::log(std::clamp(areaRatio, 0.15, 5.0))));
                const double proxTerm = crossView
                    ? 1.0
                    : (1.0 - 0.18 * std::min(1.0, cv::norm(cand.center - m_trackedCenter) / frameDiag));
                const double total = (0.72 * id.score + 0.28 * shapeSim) * areaTerm * proxTerm;
                if (total > bestScore) {
                    bestScore = total;
                    cand.score = std::clamp(id.score, 0.0, 1.0);
                    bestDet = cand;
                    bestContour = std::move(cFull);
                }
            }
            if (bestScore > 0.0 && bestDet.valid) {
                outDet = bestDet;
                outContour = std::move(bestContour);
                return true;
            }
        }
    }

    // 路径 A1：ORB 关键点几何投票重找（借鉴 untitled7，覆盖有文字/纹理的目标）
    if (redetectByOrb(bgr, outDet, outContour, crossView)) {
        return true;
    }

    // 路径 A2：非彩色目标原地倾倒/缩放的局部 Lab 背景对比兜底重找
    if (!crossView && !m_isChromatic && m_referenceFromContour && m_bgRingReliable) {
        const int radius = std::max(100, int(std::lround(std::max(m_reference.box.width, m_reference.box.height) * 1.45)));
        const cv::Rect localSearch = clampRect(
            cv::Rect(int(std::lround(m_trackedCenter.x)) - radius,
                     int(std::lround(m_trackedCenter.y)) - radius,
                     radius * 2, radius * 2), bgr.size());
        Detection d;
        std::vector<cv::Point> c;
        const int minSeg = std::max(48, int(m_reference.area * 0.18));
        if (segmentTargetInBox(bgr, localSearch, m_bgLabMean, m_bgLabSpread, true, minSeg, d, c)) {
            const double areaRatio = d.area / std::max(1.0, m_reference.area);
            if (areaRatio >= 0.25 && areaRatio <= 3.5) {
                cv::Mat localMask = cv::Mat::zeros(d.box.height, d.box.width, CV_8UC1);
                std::vector<std::vector<cv::Point>> shifted(1);
                for (const auto &pt : c) shifted[0].emplace_back(pt.x - d.box.x, pt.y - d.box.y);
                cv::drawContours(localMask, shifted, 0, cv::Scalar(255), cv::FILLED);
                const IdentityScore id = evaluateIdentity(bgr, d.box, &localMask, true, false);
                if (id.valid) {
                    d.score = std::clamp(id.score, 0.0, 1.0);
                    outDet = d;
                    outContour = std::move(c);
                    return true;
                }
            }
        }
    }

    // 路径 B：多尺度灰阶模板 Top-K 峰值 + 严格身份核验（含 90° 倾倒模板）
    if (!m_refGrayTemplate.empty() && (!crossView || m_lostFrames <= 2 || (m_lostFrames % 2 == 0))) {
        cv::Mat graySmall;
        cv::cvtColor(smallBgr, graySmall, cv::COLOR_BGR2GRAY);
        static constexpr std::array<double, 7> kScales{0.68, 0.78, 0.89, 1.00, 1.12, 1.25, 1.40};
        static constexpr std::array<double, 3> kRotScales{0.85, 1.00, 1.15};

        double bestScore = -1.0;
        cv::Rect bestBox;
        double bestScale = 1.0;
        double bestIdScore = 0.0;
        double bestAngleOffset = 0.0;

        auto searchTemplate = [&](const cv::Mat &tmpl, const double *scales, size_t nScales, double angleOffset) {
            for (size_t si = 0; si < nScales; ++si) {
                const double effScale = scales[si] * lenScale;
                const int tw = std::max(6, int(std::lround(tmpl.cols * effScale * ds)));
                const int th = std::max(6, int(std::lround(tmpl.rows * effScale * ds)));
                if (tw >= graySmall.cols || th >= graySmall.rows) continue;

                cv::Mat scaledTmpl;
                cv::resize(tmpl, scaledTmpl, cv::Size(tw, th), 0, 0,
                           effScale * ds < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);
                cv::Mat res;
                cv::matchTemplate(graySmall, scaledTmpl, res, cv::TM_CCOEFF_NORMED);

                for (int k = 0; k < 4; ++k) {
                    double maxVal = -1.0;
                    cv::Point maxLoc;
                    cv::minMaxLoc(res, nullptr, &maxVal, nullptr, &maxLoc);
                    if (maxVal < (crossView ? 0.50 : 0.56)) break;

                    const cv::Rect suppress = clampRect(
                        cv::Rect(maxLoc.x - tw / 2, maxLoc.y - th / 2, tw, th), res.size());
                    res(suppress).setTo(-1.0f);

                    const cv::Rect fullBox = clampRect(
                        cv::Rect(int(std::lround(maxLoc.x / ds)),
                                 int(std::lround(maxLoc.y / ds)),
                                 int(std::lround(tmpl.cols * effScale)),
                                 int(std::lround(tmpl.rows * effScale))),
                        bgr.size());
                    if (rotationInvariantShapeSimilarity(fullBox.size(), expectedSize) < 0.40) continue;
                    const IdentityScore id = evaluateIdentity(bgr, fullBox, nullptr, true, crossView);
                    if (!id.valid) continue;

                    const cv::Point2d cc(fullBox.x + fullBox.width * 0.5, fullBox.y + fullBox.height * 0.5);
                    const double proxTerm = crossView
                        ? 1.0
                        : (1.0 - 0.12 * std::min(1.0, cv::norm(cc - m_trackedCenter) / frameDiag));
                    const double total = (0.50 * maxVal + 0.50 * id.score) * proxTerm;
                    if (total > bestScore) {
                        bestScore = total;
                        bestBox = fullBox;
                        bestScale = effScale;
                        bestIdScore = id.score;
                        bestAngleOffset = angleOffset;
                    }
                }
            }
        };

        searchTemplate(m_refGrayTemplate, kScales.data(), kScales.size(), 0.0);
        if (bestScore <= 0.0 && !crossView) {
            cv::Mat rotCw, rotCcw;
            cv::rotate(m_refGrayTemplate, rotCw, cv::ROTATE_90_CLOCKWISE);
            cv::rotate(m_refGrayTemplate, rotCcw, cv::ROTATE_90_COUNTERCLOCKWISE);
            searchTemplate(rotCw, kRotScales.data(), kRotScales.size(), 90.0);
            searchTemplate(rotCcw, kRotScales.data(), kRotScales.size(), -90.0);
        }

        if (bestScore > 0.0 && bestBox.width >= 8 && bestBox.height >= 8) {
            if (!refineContourInBox(bgr, bestBox, outDet, outContour)) {
                outDet = detectionFromBox(bestBox, m_reference.area * bestScale * bestScale,
                                          double(bestBox.area()),
                                          m_reference.angle + bestAngleOffset,
                                          (std::abs(bestAngleOffset) > 45.0) && m_reference.angleReliable,
                                          bestIdScore);
                outContour.clear();
            }
            return true;
        }
    }

    return false;
}

QList<Detection> AutoContourDetector::initAndDetectRoi(const cv::Mat &bgr)
{
    QList<Detection> result;
    const cv::Rect rawRoi = clampRect(m_pendingRoi, bgr.size());
    m_hasPendingRoi = false;
    if (rawRoi.width < 12 || rawRoi.height < 12) return result;

    // 1. 若 YOLO26n 就绪，先识别框选区域是否属于明确语义类别（如瓶子/杯子/手机），并锚定完整外接框
    cv::Rect roi = rawRoi;
    m_yoloClassId = -1;
    m_yoloClassName.clear();
    m_yoloBoxAnchored = false;
    if (m_yoloEnabled) {
        YoloBox yoloMatch;
        if (YoloDetector::instance().classifySelection(bgr, rawRoi, yoloMatch)) {
            m_yoloClassId = yoloMatch.classId;
            m_yoloClassName = yoloMatch.className;
            m_yoloBoxAnchored = true;
            roi = yoloMatch.box;
        }
    }

    // 2. 采样 ROI 外围 12px 环形背景，辅助初始紧致轮廓提取
    const cv::Rect outer = expandRect(roi, 12, bgr.size());
    cv::Mat outerLab;
    cv::cvtColor(bgr(outer), outerLab, cv::COLOR_BGR2Lab);
    const cv::Rect innerInOuter(roi.x - outer.x, roi.y - outer.y, roi.width, roi.height);

    cv::Vec3d sumLab(0.0, 0.0, 0.0);
    int ringCount = 0;
    std::vector<cv::Vec3b> ringPixels;
    ringPixels.reserve(size_t(std::max(0, outer.area() - roi.area())));
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

    if (m_yoloBoxAnchored) {
        // YOLO 已给出完整外接框（含瓶盖与反光/白标签）：保持完整框尺寸作为参考面积，绝不被局部颜色掩膜截断
        m_reference = detectionFromBox(roi, double(roi.area()), double(roi.area()), 0.0, false, 1.0);
        m_referenceContour.clear();
        m_referenceFromContour = false;
    } else {
        Detection det;
        std::vector<cv::Point> contour;
        const int minSegArea = std::max(48, std::min(m_params.minArea, roi.area() / 8));
        bool segmented = segmentTargetInBox(bgr, roi, m_bgLabMean, m_bgLabSpread,
                                            m_bgRingReliable, minSegArea, det, contour);
        if (!segmented && m_bgRingReliable) {
            segmented = segmentTargetInBox(bgr, roi, m_bgLabMean, m_bgLabSpread,
                                           false, minSegArea, det, contour);
        }

        if (segmented && det.area >= std::max(48.0, roi.area() * 0.08)) {
            m_reference = det;
            m_referenceContour = std::move(contour);
            m_referenceFromContour = true;
        } else {
            m_reference = detectionFromBox(roi, double(roi.area()), double(roi.area()), 0.0, false, 1.0);
            m_referenceContour.clear();
            m_referenceFromContour = false;
        }
    }

    // 3. 提取紧致灰阶模板 + ORB 描述子 + 核心前景 HSV/Lab 色度签名
    cv::cvtColor(bgr(m_reference.box), m_refGrayTemplate, cv::COLOR_BGR2GRAY);
    buildOrbReference(bgr, m_reference.box);

    cv::Mat refMask;
    if (m_referenceFromContour && !m_referenceContour.empty()) {
        refMask = cv::Mat::zeros(m_reference.box.height, m_reference.box.width, CV_8UC1);
        std::vector<std::vector<cv::Point>> shifted(1);
        shifted[0].reserve(m_referenceContour.size());
        for (const auto &pt : m_referenceContour)
            shifted[0].emplace_back(pt.x - m_reference.box.x, pt.y - m_reference.box.y);
        cv::drawContours(refMask, shifted, 0, cv::Scalar(255), cv::FILLED);
        cv::Mat eroded;
        static const cv::Mat kErode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::erode(refMask, eroded, kErode);
        if (cv::countNonZero(eroded) >= 32) refMask = eroded;
    }

    const ColorSignature refSig = refMask.empty()
        ? extractColorSignature(bgr(innerCoreRect(m_reference.box, bgr.size())))
        : extractColorSignature(bgr(m_reference.box), refMask);
    m_refHsHist = refSig.hsHist;
    m_refLabHist = refSig.labHist;
    m_refCoreLab = refSig.coreLab;
    m_refChromaticFrac = refSig.chromaticFrac;
    m_isChromatic = (m_refChromaticFrac >= 0.22 && !m_refHsHist.empty());

    // 仅在非 YOLO 锚定且初始未贴合轮廓的彩色目标上尝试 HSV 细化；若细化后只覆盖很小局部则保留完整框
    if (!m_yoloBoxAnchored && !m_referenceFromContour && m_isChromatic && !m_refHsHist.empty()) {
        Detection chromDet;
        std::vector<cv::Point> chromContour;
        if (refineContourInBox(bgr, roi, chromDet, chromContour)
            && chromDet.area >= std::max(48.0, roi.area() * 0.28)
            && std::max(chromDet.box.width, chromDet.box.height) >= std::max(roi.width, roi.height) * 0.72) {
            m_reference = chromDet;
            m_referenceContour = std::move(chromContour);
            m_referenceFromContour = true;
            cv::cvtColor(bgr(m_reference.box), m_refGrayTemplate, cv::COLOR_BGR2GRAY);
            buildOrbReference(bgr, m_reference.box);
            cv::Mat tightMask = cv::Mat::zeros(m_reference.box.height, m_reference.box.width, CV_8UC1);
            std::vector<std::vector<cv::Point>> shifted(1);
            for (const auto &pt : m_referenceContour)
                shifted[0].emplace_back(pt.x - m_reference.box.x, pt.y - m_reference.box.y);
            cv::drawContours(tightMask, shifted, 0, cv::Scalar(255), cv::FILLED);
            const ColorSignature tightSig = extractColorSignature(bgr(m_reference.box), tightMask);
            if (!tightSig.hsHist.empty() && !tightSig.labHist.empty()) {
                m_refHsHist = tightSig.hsHist;
                m_refLabHist = tightSig.labHist;
                m_refCoreLab = tightSig.coreLab;
                m_refChromaticFrac = tightSig.chromaticFrac;
                m_isChromatic = (m_refChromaticFrac >= 0.22);
            }
        }
    }

    // YOLO 锚定目标同步提取首帧内部轮廓的真实视角倾角作为参考角（不改变完整框尺寸），
    // 消除首帧硬编码 90° 与后续真实斜角 82° 之间的固定 8° 偏置
    if (m_yoloBoxAnchored) {
        Detection initRefined;
        std::vector<cv::Point> initContour;
        if (refineContourInBox(bgr, m_reference.box, initRefined, initContour)) {
            const int rMax = std::max(initRefined.box.width, initRefined.box.height);
            const int bMax = std::max(m_reference.box.width, m_reference.box.height);
            if (initRefined.angleReliable && rMax >= bMax * 0.68) {
                m_reference.angle = initRefined.angle;
                m_reference.angleReliable = true;
            }
        }
    }

    // 4. 初始化 CSRT + 卡尔曼 + 平滑框起点
    m_crossHsHist = smoothHsHistForCrossView(m_refHsHist);
    initCsrtTracker(bgr, m_reference.box);
    initKalmanAt(m_reference.center);
    m_smoothedBox = cv::Rect2d(m_reference.box.x, m_reference.box.y, m_reference.box.width, m_reference.box.height);
    m_hasSmoothedBox = true;
    m_smoothedAngle = m_reference.angle;
    m_hasSmoothedAngle = m_reference.angleReliable;
    m_lastYoloAnchorMs = monitorNowMs();
    m_lastReliableDet = m_reference;
    m_lastReliableContour = m_referenceContour;

    m_roiMode = true;
    m_hasTrackedTarget = true;
    m_lostFrames = 0;
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
    if (!m_reference.valid) return result;

    if (m_kalmanReady) m_kalman.predict();
    ++m_framesSinceCsrtReinit;

    const qint64 nowMs = monitorNowMs();
    Detection out;
    std::vector<cv::Point> contour;
    bool found = false;
    bool snapBox = false;
    bool needReinitCsrt = false;

    // 1. 优先运行 CSRT 多通道特征跟踪（HOG + Color-Names + RGB）
    if (m_csrtActive && m_csrt) {
        cv::Mat smallBgr;
        if (m_csrtScale < 1.0) {
            cv::resize(bgr, smallBgr, cv::Size(), m_csrtScale, m_csrtScale, cv::INTER_AREA);
        } else {
            smallBgr = bgr;
        }
        cv::Rect smallBox;
        bool ok = false;
        try {
            ok = m_csrt->update(smallBgr, smallBox);
        } catch (const cv::Exception &) {
            ok = false;
        }
        if (ok && smallBox.width >= 6 && smallBox.height >= 6) {
            const cv::Rect fullBox = clampRect(
                cv::Rect(int(std::lround(smallBox.x / m_csrtScale)),
                         int(std::lround(smallBox.y / m_csrtScale)),
                         int(std::lround(smallBox.width / m_csrtScale)),
                         int(std::lround(smallBox.height / m_csrtScale))),
                bgr.size());
            const double boxArea = double(fullBox.area());
            const double refBoxArea = std::max(1.0, double(m_reference.box.area()));
            if (fullBox.width >= 8 && fullBox.height >= 8
                && boxArea >= refBoxArea * 0.14 && boxArea <= refBoxArea * 5.5) {
                const IdentityScore id = evaluateIdentity(bgr, fullBox, nullptr, false);

                if (m_yoloBoxAnchored) {
                    // YOLO 语义锚定目标（如瓶子）：外接框由 CSRT + 周期性 YOLO 完整框主导，绝不让反光/白标签把框缩成局部
                    if (id.valid) {
                        cv::Rect targetBox = fullBox;
                        double finalScore = id.score;
                        bool anchorSnapped = false;
                        // 每 ~450 ms 用 YOLO26n 校准一次完整目标框（含 90° 横躺补检），防止 CSRT 尺度缓慢漂移
                        if (m_yoloEnabled && m_yoloClassId >= 0 && YoloDetector::instance().isReady()
                            && (m_lastYoloAnchorMs < 0 || nowMs - m_lastYoloAnchorMs >= kYoloAnchorIntervalMs)) {
                            m_lastYoloAnchorMs = nowMs;
                            const auto yBoxes = YoloDetector::instance().detect(bgr, m_yoloClassId, true);
                            double bestAnchorScore = -1.0;
                            cv::Rect bestAnchorBox;
                            for (const auto &yb : yBoxes) {
                                if (yb.classId != m_yoloClassId) continue;
                                const double iou = rectIoU(yb.box, fullBox);
                                const cv::Point2d yc(yb.box.x + yb.box.width * 0.5, yb.box.y + yb.box.height * 0.5);
                                const cv::Point2d fc(fullBox.x + fullBox.width * 0.5, fullBox.y + fullBox.height * 0.5);
                                const double cDist = cv::norm(yc - fc);
                                if (iou < 0.20 && cDist > std::max(fullBox.width, fullBox.height) * 0.85) continue;
                                const IdentityScore yId = evaluateIdentity(bgr, yb.box, nullptr, false);
                                if (!yId.valid) continue;
                                const double sc = yId.score * 0.65 + iou * 0.35;
                                if (sc > bestAnchorScore) {
                                    bestAnchorScore = sc;
                                    bestAnchorBox = yb.box;
                                }
                            }
                            if (bestAnchorScore > 0.0) {
                                const double driftIou = rectIoU(bestAnchorBox, fullBox);
                                const double yAspect = double(bestAnchorBox.height) / std::max(1, bestAnchorBox.width);
                                const double fAspect = double(fullBox.height) / std::max(1, fullBox.width);
                                const bool aspectFlipped = (yAspect / std::max(1e-6, fAspect) > 1.45)
                                                        || (yAspect / std::max(1e-6, fAspect) < 0.69);
                                const cv::Point2d yc(bestAnchorBox.x + bestAnchorBox.width * 0.5,
                                                     bestAnchorBox.y + bestAnchorBox.height * 0.5);
                                const cv::Point2d fc(fullBox.x + fullBox.width * 0.5,
                                                     fullBox.y + fullBox.height * 0.5);
                                const double anchorDist = cv::norm(yc - fc);
                                const double scaleDim = std::max(20.0, double(std::max(fullBox.width, fullBox.height)));
                                const double wDiff = std::abs(bestAnchorBox.width - fullBox.width) / std::max(1.0, double(fullBox.width));
                                const double hDiff = std::abs(bestAnchorBox.height - fullBox.height) / std::max(1.0, double(fullBox.height));
                                // CSRT 与 YOLO 已经高度一致时保持 CSRT 不拉扯，杜绝每 450ms 的来回微抖
                                const bool alreadyAligned = !aspectFlipped && (driftIou >= 0.72)
                                    && (anchorDist <= std::max(10.0, scaleDim * 0.08))
                                    && (wDiff <= 0.14) && (hDiff <= 0.14);
                                if (!alreadyAligned) {
                                    targetBox = bestAnchorBox;
                                    finalScore = std::max(finalScore, bestAnchorScore);
                                    if (m_framesSinceCsrtReinit >= 4 && (driftIou < 0.65 || aspectFlipped)) {
                                        needReinitCsrt = true;
                                        anchorSnapped = aspectFlipped;
                                    }
                                }
                            }
                        }
                        const cv::Rect smoothBox = smoothOutputBox(targetBox, bgr.size(), anchorSnapped);
                        out = detectionFromBox(smoothBox, m_reference.area, refBoxArea,
                                               m_reference.angle, m_reference.angleReliable, finalScore);
                        // 仅当轮廓覆盖主体（未截断）时借用其精细主轴倾角并做角度死区平滑
                        Detection refined;
                        std::vector<cv::Point> c;
                        if (refineContourInBox(bgr, smoothBox, refined, c)) {
                            const int rMax = std::max(refined.box.width, refined.box.height);
                            const int sMax = std::max(smoothBox.width, smoothBox.height);
                            if (refined.angleReliable && rMax >= sMax * 0.68) {
                                out.angle = smoothOutputAngle(refined.angle, anchorSnapped);
                                out.angleReliable = true;
                                contour = std::move(c);
                            }
                        }
                        found = true;
                        m_unreliableCount = 0;
                    }
                } else {
                    // 传统非 YOLO 目标：局部轮廓细化必须防“局部截断缩框棘轮”
                    Detection refined;
                    std::vector<cv::Point> refinedContour;
                    if (refineContourInBox(bgr, fullBox, refined, refinedContour)) {
                        cv::Mat localMask = cv::Mat::zeros(refined.box.height, refined.box.width, CV_8UC1);
                        std::vector<std::vector<cv::Point>> shifted(1);
                        for (const auto &pt : refinedContour)
                            shifted[0].emplace_back(pt.x - refined.box.x, pt.y - refined.box.y);
                        cv::drawContours(localMask, shifted, 0, cv::Scalar(255), cv::FILLED);
                        const IdentityScore refinedId = evaluateIdentity(bgr, refined.box, &localMask, !id.valid);
                        if (refinedId.valid) {
                            const int rMax = std::max(refined.box.width, refined.box.height);
                            const int rMin = std::min(refined.box.width, refined.box.height);
                            const int bMax = std::max(fullBox.width, fullBox.height);
                            const int bMin = std::min(fullBox.width, fullBox.height);
                            const double boxAspect = double(fullBox.height) / std::max(1, fullBox.width);
                            const double outAspect = double(refined.box.height) / std::max(1, refined.box.width);
                            const double aspectRatioDiff = outAspect / std::max(1e-6, boxAspect);
                            const bool flipped90 = (aspectRatioDiff > 1.45 || aspectRatioDiff < 0.69)
                                && (rMax >= std::max(m_reference.box.width, m_reference.box.height) * 0.68);
                            // 当 CSRT 自身有效但局部颜色掩膜明显偏小且非 90° 翻转时，拒绝缩框（防标签截断）
                            const bool partialShrinkClip = id.valid && !flipped90
                                && m_isChromatic && (m_refChromaticFrac < 0.88)
                                && (rMax < bMax * 0.78 || rMin < bMin * 0.70);

                            if (!partialShrinkClip) {
                                out = refined;
                                out.score = std::clamp(refinedId.score, 0.0, 1.0);
                                contour = std::move(refinedContour);
                                if (m_boxSmoothingEnabled && !flipped90) {
                                    const cv::Rect sBox = smoothOutputBox(out.box, bgr.size(), false);
                                    const double ratio = double(std::max(1, sBox.area())) / std::max(1.0, double(out.box.area()));
                                    out.box = sBox;
                                    out.center = cv::Point2d(sBox.x + sBox.width * 0.5, sBox.y + sBox.height * 0.5);
                                    out.area = std::max(1.0, out.area * ratio);
                                    out.aspectRatio = double(sBox.height) / std::max(1, sBox.width);
                                } else {
                                    (void)smoothOutputBox(out.box, bgr.size(), true);
                                }
                                found = true;
                                m_unreliableCount = 0;
                                // 仅在真正 90° 倾倒翻转或大幅偏离（且非局部缩框）时才重初始化 CSRT，彻底杜绝缩框棘轮
                                const double iou = rectIoU(out.box, fullBox);
                                if (m_framesSinceCsrtReinit >= 4 && (flipped90 || (iou < 0.42 && rMax >= bMax * 0.82))) {
                                    needReinitCsrt = true;
                                }
                            }
                        }
                    }
                    if (!found && id.valid) {
                        const cv::Rect sBox = smoothOutputBox(fullBox, bgr.size(), false);
                        out = detectionFromBox(sBox, m_reference.area, refBoxArea,
                                               m_reference.angle, m_reference.angleReliable, id.score);
                        contour.clear();
                        found = true;
                        m_unreliableCount = 0;
                    }
                }

                // 借鉴 untitled7::updateFeatureTracker：单/双帧反光或模糊校验不通过时短时 HOLD 上一可靠框，避免频繁丢锁跳变
                if (!found && m_boxSmoothingEnabled && m_lastReliableDet.valid
                    && id.score >= kHardRejectIdentityScore
                    && m_unreliableCount + 1 < kUnreliableHoldLimit) {
                    ++m_unreliableCount;
                    out = m_lastReliableDet;
                    contour = m_lastReliableContour;
                    found = true;
                }
            }
        }
        if (!found) {
            m_csrt.release();
            m_csrtActive = false;
            m_unreliableCount = 0;
        }
    }

    // 2. 丢失/大幅位移/遮挡恢复 或 跨视角初次搜索（带限频 + 连续帧空间确认）
    if (!found) {
        const bool throttled = (m_recoveryThrottleMs > 0)
            && (m_lastRecoveryMs >= 0)
            && (nowMs >= m_lastRecoveryMs)
            && (nowMs - m_lastRecoveryMs < m_recoveryThrottleMs);
        if (!throttled) {
            m_lastRecoveryMs = nowMs;
            Detection cand;
            std::vector<cv::Point> candContour;
            if (redetectTarget(bgr, cand, candContour, m_crossViewSearching)) {
                const int reqFrames = m_crossViewSearching ? m_crossViewConfirmFrames : m_lostRecoveryConfirmFrames;
                if (confirmRecoveryCandidate(cand.box, reqFrames)) {
                    out = std::move(cand);
                    contour = std::move(candContour);
                    found = true;
                    snapBox = true;
                    needReinitCsrt = true;
                    if (m_crossViewSearching) {
                        m_crossViewSearching = false;
                        m_reference = out;
                        m_referenceContour = contour;
                        m_referenceFromContour = !contour.empty() && !m_yoloBoxAnchored;
                        const cv::Rect clampedBox = clampRect(out.box, bgr.size());
                        if (clampedBox.width >= 8 && clampedBox.height >= 8) {
                            cv::cvtColor(bgr(clampedBox), m_refGrayTemplate, cv::COLOR_BGR2GRAY);
                            buildOrbReference(bgr, clampedBox);
                            cv::Mat localMask;
                            if (!contour.empty()) {
                                localMask = cv::Mat::zeros(clampedBox.height, clampedBox.width, CV_8UC1);
                                std::vector<std::vector<cv::Point>> shifted(1);
                                for (const auto &pt : contour)
                                    shifted[0].emplace_back(pt.x - clampedBox.x, pt.y - clampedBox.y);
                                cv::drawContours(localMask, shifted, 0, cv::Scalar(255), cv::FILLED);
                                cv::Mat eroded;
                                static const cv::Mat kErode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
                                cv::erode(localMask, eroded, kErode);
                                if (cv::countNonZero(eroded) >= 32) localMask = eroded;
                            }
                            const ColorSignature localSig = localMask.empty()
                                ? extractColorSignature(bgr(innerCoreRect(clampedBox, bgr.size())))
                                : extractColorSignature(bgr(clampedBox), localMask);
                            m_refCoreLab = 0.65 * localSig.coreLab + 0.35 * m_refCoreLab;
                            m_refChromaticFrac = 0.65 * localSig.chromaticFrac + 0.35 * m_refChromaticFrac;
                            if (!localSig.hsHist.empty() && !m_refHsHist.empty() && localSig.hsHist.size() == m_refHsHist.size()) {
                                m_refHsHist = 0.65f * localSig.hsHist + 0.35f * m_refHsHist;
                                const double s = cv::sum(m_refHsHist)[0];
                                if (s > 0.0) m_refHsHist /= float(s);
                                m_crossHsHist = smoothHsHistForCrossView(m_refHsHist);
                            }
                            if (!localSig.labHist.empty() && !m_refLabHist.empty()) {
                                m_refLabHist = 0.65f * localSig.labHist + 0.35f * m_refLabHist;
                                const double s = cv::sum(m_refLabHist)[0];
                                if (s > 0.0) m_refLabHist /= float(s);
                            }
                        }
                        initKalmanAt(out.center);
                        m_hasTrackedTarget = true;
                        m_lastYoloAnchorMs = nowMs;
                    }
                }
            } else {
                resetRecoveryCandidate();
            }
        }
    }

    auto buildMask = [&](const Detection &d, const std::vector<cv::Point> &c) {
        if (!m_computeMask) return;
        m_mask = cv::Mat::zeros(bgr.rows, bgr.cols, CV_8UC1);
        if (!c.empty()) {
            std::vector<std::vector<cv::Point>> draw{c};
            cv::drawContours(m_mask, draw, 0, cv::Scalar(255), cv::FILLED);
        } else if (d.valid) {
            cv::rectangle(m_mask, d.box, cv::Scalar(255), cv::FILLED);
        }
    };

    if (!found) {
        ++m_lostFrames;
        if (m_computeMask) m_mask = cv::Mat::zeros(bgr.rows, bgr.cols, CV_8UC1);
        return result;
    }

    if (snapBox) {
        (void)smoothOutputBox(out.box, bgr.size(), true);
        if (out.angleReliable) (void)smoothOutputAngle(out.angle, true);
    }
    m_lostFrames = 0;
    m_trackedCenter = out.center;
    m_trackedArea = out.area;
    m_trackedScale = std::sqrt(out.area / std::max(1.0, m_reference.area));
    m_lastReliableDet = out;
    m_lastReliableContour = contour;
    if (m_kalmanReady) {
        cv::Mat meas = (cv::Mat_<float>(2, 1) << float(out.center.x), float(out.center.y));
        m_kalman.correct(meas);
    }
    if (needReinitCsrt) {
        initCsrtTracker(bgr, out.box);
    }
    buildMask(out, contour);
    result.append(out);
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

    if (!m_hasPickedPoint && !m_hasTrackedTarget && !m_computeMask) {
        m_mask.release();
        return result;
    }

    cv::Mat gray, blurred;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.2);

    cv::Mat edges;
    if (m_params.autoThreshold) {
        const double med = computeMedian(blurred);
        const double lowThresh = std::max(20.0, med * 0.30);
        const double highThresh = std::min(200.0, med * 0.75);
        cv::Canny(blurred, edges, lowThresh, highThresh);
    } else {
        cv::Canny(blurred, edges, m_params.cannyLow, m_params.cannyHigh);
    }

    const int kSize = std::max(3, m_params.morphKernelSize | 1);
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kSize, kSize));
    cv::morphologyEx(edges, m_mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(m_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!m_hasPickedPoint && !m_hasTrackedTarget)
        return result;
    const bool picking = m_hasPickedPoint;
    m_hasPickedPoint = false;
    struct Candidate {
        double score = 0.0;
        Detection det;
        size_t contourIndex = 0;
    };
    std::vector<Candidate> candidates;

    for (size_t i = 0; i < contours.size(); ++i) {
        const auto &contour = contours[i];
        const double area = cv::contourArea(contour);
        if (area < std::max(1, m_params.minArea)) continue;

        Detection d = detectionFromContour(contour);
        if (!d.valid) continue;

        double score = 0.0;
        if (picking) {
            if (cv::pointPolygonTest(contour, m_pickedPoint, false) < 0) continue;
            score = 1.0 / d.area;
        } else {
            const double diagonal = std::hypot(m_reference.box.width, m_reference.box.height);
            const double referenceDistance = cv::norm(d.center - m_reference.center);
            const double distToLast = cv::norm(d.center - m_trackedCenter);
            const double areaRatio = d.area / std::max(1.0, m_reference.area);
            const double shapeDistance = cv::matchShapes(m_referenceContour, contour, cv::CONTOURS_MATCH_I1, 0);
            if (referenceDistance > std::max(30.0, diagonal * 0.8)
                || distToLast > std::max(25.0, diagonal * 0.5)
                || areaRatio < 0.4 || areaRatio > 2.5 || shapeDistance > 0.30)
                continue;
            score = 1.0 / (1.0 + distToLast / std::max(1.0, diagonal)
                           + std::abs(std::log(areaRatio)) + shapeDistance);
        }
        candidates.push_back({score, d, i});
    }

    if (candidates.empty()) return result;

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &a, const Candidate &b) { return a.score > b.score; });

    const Detection &best = candidates.front().det;
    if (picking) {
        m_reference = best;
        m_referenceContour = contours[candidates.front().contourIndex];
    }
    m_trackedCenter = best.center;
    m_trackedArea = best.area;
    m_hasTrackedTarget = true;
    result.append(best);
    return result;
}
