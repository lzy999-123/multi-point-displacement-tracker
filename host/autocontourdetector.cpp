#include "autocontourdetector.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr int kSatFloor = 38;
constexpr int kValFloor = 35;

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

// 仅在有明显色彩（S>=kSatFloor, V>=kValFloor）的像素上统计 2D Hue-Sat 直方图与主色 Lab
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
    // calcBackProject 按直方图归一化权重映射概率，放大至 0~255
    cv::Mat scaledHist = hsHist * 255.0f;
    double maxVal = 0.0;
    cv::minMaxLoc(scaledHist, nullptr, &maxVal);
    if (maxVal > 1e-3) scaledHist *= float(255.0 / maxVal);
    cv::calcBackProject(&hsv, 1, channels, scaledHist, prob, ranges, 1.0);

    // 抹去低饱和度与暗区背景，杜绝白/灰/黑物体干扰彩色目标
    for (int r = 0; r < hsv.rows; ++r) {
        const auto *hRow = hsv.ptr<cv::Vec3b>(r);
        auto *pRow = prob.ptr<uchar>(r);
        for (int c = 0; c < hsv.cols; ++c) {
            if (hRow[c][1] < kSatFloor || hRow[c][2] < kValFloor) pRow[c] = 0;
        }
    }
    return prob;
}

// 对 24x12 的 Hue-Sat 直方图做 1-bin 循环色相轻度平滑，用于吸收不同摄像头间的微小白平衡差异
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
    m_crossViewSearching = false;
    m_sourceFrameSize = cv::Size();
    m_referenceFromContour = false;
    m_csrt.release();
    m_csrtActive = false;
    m_csrtScale = 1.0;
    m_lostFrames = 0;
    m_refGrayTemplate.release();
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

TargetSignature AutoContourDetector::exportSignature(const cv::Mat &bgr) const
{
    TargetSignature sig;
    if (!m_reference.valid || bgr.empty()) return sig;

    sig.refBoxSize = m_reference.box.size();
    sig.refArea = std::max(1.0, m_reference.area);
    sig.sourceFrameSize = bgr.size();
    sig.aspectRatio = m_reference.aspectRatio;

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

    // 兼容单击点选模式：从当前帧直接提取核心前景签名
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
    m_refHsHist = sig.hsHist.clone();
    m_crossHsHist = smoothHsHistForCrossView(m_refHsHist);
    m_refLabHist = sig.labHist.clone();
    m_refCoreLab = sig.coreLab;
    m_refChromaticFrac = sig.chromaticFrac;
    m_refGrayTemplate = sig.grayTemplate.clone();

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
    params.filter_lr = 0.015f;     // 保守在线学习率，减少短暂遮挡或经过同类物体时的模型漂移
    params.histogram_lr = 0.02f;
    params.psr_threshold = 0.035f;

    try {
        m_csrt = cv::TrackerCSRT::create(params);
        m_csrt->init(smallBgr, smallBox);
        m_csrtActive = true;
    } catch (const cv::Exception &) {
        m_csrt.release();
        m_csrtActive = false;
    }
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
        // 彩色目标门控聚焦亮度无关的 Hue-Sat 分布与 CIE Lab 色度 (a,b)，
        // 避免不同机位或阴影导致的亮度 L 跨桶误拒，同时严格排除蓝/绿/红/白等异色干扰物
        const double minChromFrac = std::max(0.08, m_refChromaticFrac * (crossView ? 0.30 : (strictForRedetect ? 0.42 : 0.30)));
        const double minHsSim = crossView ? 0.42 : (strictForRedetect ? 0.48 : 0.38);
        const double maxAbDist = crossView ? 42.0 : (strictForRedetect ? 32.0 : 40.0);
        if (curr.chromaticFrac < minChromFrac || res.hsSim < minHsSim || res.abDist > maxAbDist) {
            return res;
        }
        res.valid = true;
        res.score = 0.55 * res.hsSim + 0.30 * std::max(0.0, 1.0 - res.abDist / 45.0) + 0.15 * res.labSim;
        return res;
    }

    // 非彩色（黑白/灰/金属质感）目标：使用 Lab 分布 + 防串彩色物体 + 灰度相关性校验
    const double maxChromForAchromatic = strictForRedetect ? 0.38 : 0.48;
    const double minLabSim = crossView ? 0.40 : (strictForRedetect ? 0.52 : 0.42);
    if (curr.chromaticFrac > maxChromForAchromatic || res.labSim < minLabSim) {
        return res;
    }
    double grayCorr = 0.5;
    if (!m_refGrayTemplate.empty()) {
        cv::Mat gPatch, gResized;
        cv::cvtColor(bgr(clamped), gPatch, cv::COLOR_BGR2GRAY);
        cv::resize(gPatch, gResized, m_refGrayTemplate.size(), 0, 0, cv::INTER_AREA);
        cv::Mat corr;
        cv::matchTemplate(gResized, m_refGrayTemplate, corr, cv::TM_CCOEFF_NORMED);
        grayCorr = double(corr.at<float>(0, 0));
        if (grayCorr < (crossView ? 0.38 : (strictForRedetect ? 0.45 : 0.30))) return res;
    }
    res.valid = true;
    res.score = 0.55 * res.labSim + 0.45 * std::clamp(grayCorr, 0.0, 1.0);
    return res;
}

bool AutoContourDetector::refineContourInBox(const cv::Mat &bgr, const cv::Rect &box,
                                             Detection &outDet, std::vector<cv::Point> &outContour) const
{
    const cv::Rect win = expandRect(box, 10, bgr.size());
    if (win.width < 10 || win.height < 10) return false;

    const int minSegArea = std::max(36, int(m_reference.area * 0.22));
    // 1. 优先对彩色目标用 HSV 反投影掩膜提取轮廓（支持带中间白色标签的瓶身上下段合并）
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
            const double pieceMinArea = std::max(24.0, double(minSegArea) * 0.20);

            std::vector<size_t> matchedIdx;
            double totalPieceArea = 0.0;
            for (size_t i = 0; i < contours.size(); ++i) {
                const double area = cv::contourArea(contours[i]);
                if (area < pieceMinArea) continue;
                const cv::Rect cb = cv::boundingRect(contours[i]);
                const cv::Rect inter = cb & localBox;
                if (inter.area() <= 0) continue;
                // 仅吸收主体位于当前跟踪框内的色块片段，避免吸入相邻物体的边缘
                if (double(inter.area()) / std::max(1.0, double(cb.area())) < 0.45) continue;
                matchedIdx.push_back(i);
                totalPieceArea += area;
            }

            if (!matchedIdx.empty() && totalPieceArea >= double(minSegArea) * 0.70) {
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
                const bool clippedLabeledTarget = (m_refChromaticFrac < 0.88)
                    && (d.box.height < box.height * 0.62 || d.box.width < box.width * 0.62);
                if (d.valid && !clippedLabeledTarget
                    && d.area >= boxArea * 0.22 && d.area <= boxArea * 1.95) {
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
            if (d.area >= boxArea * 0.25 && d.area <= boxArea * 1.90) {
                outDet = d;
                outContour = std::move(c);
                return true;
            }
        }
    }
    return false;
}

bool AutoContourDetector::redetectTarget(const cv::Mat &bgr, Detection &outDet,
                                         std::vector<cv::Point> &outContour, bool crossView)
{
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

    // 路径 A：彩色目标（如黄色瓶子）—— HSV 颜色反投影全图搜索，直接过滤掉所有不同颜色的瓶子！
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

            const double minAreaSmall = std::max(crossView ? 18.0 : 24.0,
                                                 expectedArea * ds * ds * (crossView ? 0.14 : 0.22));
            const double maxAreaSmall = std::min(double(smallBgr.cols * smallBgr.rows) * 0.65,
                                                 expectedArea * ds * ds * (crossView ? 5.0 : 3.2));
            const double pieceAreaSmall = std::max(10.0, minAreaSmall * 0.22);
            const double refWSmall = std::max(8.0, m_reference.box.width * lenScale * ds);
            const double refHSmall = std::max(8.0, m_reference.box.height * lenScale * ds);

            // 收集有效色块片段，并将被瓶身白色标签等隔开的同一目标上下/左右片段合并为整体候选
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

            const double maxGapX = std::max(6.0, refWSmall * (crossView ? 0.36 : 0.28));
            const double maxGapY = std::max(6.0, refHSmall * (crossView ? 0.52 : 0.42));
            const double maxUnionW = refWSmall * (crossView ? 2.20 : 1.65) + 6.0;
            const double maxUnionH = refHSmall * (crossView ? 2.20 : 1.65) + 6.0;

            for (size_t i = 0; i < pieces.size(); ++i) {
                for (size_t j = i + 1; j < pieces.size(); ++j) {
                    const cv::Rect &a = pieces[i].box;
                    const cv::Rect &b = pieces[j].box;
                    const int gapX = std::max(0, std::max(a.x, b.x) - std::min(a.x + a.width, b.x + b.width));
                    const int gapY = std::max(0, std::max(a.y, b.y) - std::min(a.y + a.height, b.y + b.height));
                    if (gapX > maxGapX || gapY > maxGapY) continue;
                    const cv::Rect u = a | b;
                    if (u.width > maxUnionW || u.height > maxUnionH) continue;
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
                    if (unionBox.width > maxUnionW || unionBox.height > maxUnionH) continue;
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

                // 生成该候选的精确前景掩膜进行色度身份核验
                cv::Mat localMask = cv::Mat::zeros(cand.box.height, cand.box.width, CV_8UC1);
                std::vector<std::vector<cv::Point>> shifted(1);
                shifted[0].reserve(cFull.size());
                for (const auto &pt : cFull) shifted[0].emplace_back(pt.x - cand.box.x, pt.y - cand.box.y);
                cv::drawContours(localMask, shifted, 0, cv::Scalar(255), cv::FILLED);

                const IdentityScore id = evaluateIdentity(bgr, cand.box, &localMask, true, crossView);
                if (!id.valid) continue;

                const double areaRatio = cand.area / expectedArea;
                const double areaTerm = std::exp((crossView ? -0.35 : -0.7) * std::abs(std::log(std::clamp(areaRatio, 0.15, 5.0))));
                // 跨视角初次搜索不偏向源摄像头的 (x,y) 位置
                const double proxTerm = crossView
                    ? 1.0
                    : (1.0 - 0.18 * std::min(1.0, cv::norm(cand.center - m_trackedCenter) / frameDiag));
                const double total = id.score * areaTerm * proxTerm;
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

    // 路径 B：多尺度灰阶模板 Top-K 峰值 + 严格身份核验（覆盖非彩色目标与低饱和纹理物体）
    if (!m_refGrayTemplate.empty() && (!crossView || m_lostFrames <= 2 || (m_lostFrames % 2 == 0))) {
        cv::Mat graySmall;
        cv::cvtColor(smallBgr, graySmall, cv::COLOR_BGR2GRAY);
        static constexpr std::array<double, 7> kScales{0.68, 0.78, 0.89, 1.00, 1.12, 1.25, 1.40};

        double bestScore = -1.0;
        cv::Rect bestBox;
        double bestScale = 1.0;
        double bestIdScore = 0.0;

        for (double scale : kScales) {
            const double effScale = scale * lenScale;
            const int tw = std::max(6, int(std::lround(m_refGrayTemplate.cols * effScale * ds)));
            const int th = std::max(6, int(std::lround(m_refGrayTemplate.rows * effScale * ds)));
            if (tw >= graySmall.cols || th >= graySmall.rows) continue;

            cv::Mat scaledTmpl;
            cv::resize(m_refGrayTemplate, scaledTmpl, cv::Size(tw, th), 0, 0,
                       effScale * ds < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);
            cv::Mat res;
            cv::matchTemplate(graySmall, scaledTmpl, res, cv::TM_CCOEFF_NORMED);

            // 最多检验前 4 个非极大抑制峰值，防止单一灰度干扰峰挡住真正目标
            for (int k = 0; k < 4; ++k) {
                double maxVal = -1.0;
                cv::Point maxLoc;
                cv::minMaxLoc(res, nullptr, &maxVal, nullptr, &maxLoc);
                if (maxVal < (crossView ? 0.50 : 0.56)) break;

                // 抑制该峰值邻域，便于查看次优峰
                const cv::Rect suppress = clampRect(
                    cv::Rect(maxLoc.x - tw / 2, maxLoc.y - th / 2, tw, th), res.size());
                res(suppress).setTo(-1.0f);

                const cv::Rect fullBox = clampRect(
                    cv::Rect(int(std::lround(maxLoc.x / ds)),
                             int(std::lround(maxLoc.y / ds)),
                             int(std::lround(m_refGrayTemplate.cols * effScale)),
                             int(std::lround(m_refGrayTemplate.rows * effScale))),
                    bgr.size());
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
                }
            }
        }

        if (bestScore > 0.0 && bestBox.width >= 8 && bestBox.height >= 8) {
            if (!refineContourInBox(bgr, bestBox, outDet, outContour)) {
                outDet = Detection();
                outDet.box = bestBox;
                outDet.rotBox = cv::RotatedRect(
                    cv::Point2f(bestBox.x + bestBox.width * 0.5f, bestBox.y + bestBox.height * 0.5f),
                    cv::Size2f(float(bestBox.width), float(bestBox.height)), 0.0f);
                outDet.center = cv::Point2d(bestBox.x + bestBox.width * 0.5, bestBox.y + bestBox.height * 0.5);
                outDet.area = m_reference.area * bestScale * bestScale;
                outDet.aspectRatio = double(bestBox.height) / std::max(1, bestBox.width);
                outDet.angle = m_reference.angle;
                outDet.angleReliable = false;
                outDet.score = std::clamp(bestIdScore, 0.0, 1.0);
                outDet.valid = true;
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
    const cv::Rect roi = clampRect(m_pendingRoi, bgr.size());
    m_hasPendingRoi = false;
    if (roi.width < 12 || roi.height < 12) return result;

    // 采样 ROI 外围 12px 环形背景，辅助初始紧致轮廓提取
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

    // 紧致无背景灰阶模板（不再外扩 4px 污染背景）
    cv::cvtColor(bgr(m_reference.box), m_refGrayTemplate, cv::COLOR_BGR2GRAY);

    // 提取核心前景颜色签名（优先用腐蚀后的前景轮廓掩膜，彻底排除框外背景像素）
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

    // 初始化 CSRT 多通道特征跟踪器 + 2D 卡尔曼滤波器
    m_crossHsHist = smoothHsHistForCrossView(m_refHsHist);
    initCsrtTracker(bgr, m_reference.box);
    initKalmanAt(m_reference.center);

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

    Detection out;
    std::vector<cv::Point> contour;
    bool found = false;
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
                && boxArea >= refBoxArea * 0.20 && boxArea <= refBoxArea * 4.0) {
                // 核心色度身份门控：拒绝漂移到其他颜色瓶子或背景上的伪命中
                const IdentityScore id = evaluateIdentity(bgr, fullBox, nullptr, false);
                if (id.valid) {
                    if (refineContourInBox(bgr, fullBox, out, contour)) {
                        // 对细化后的轮廓再做一次身份核验，确保没吸到相邻异色物体
                        cv::Mat localMask = cv::Mat::zeros(out.box.height, out.box.width, CV_8UC1);
                        std::vector<std::vector<cv::Point>> shifted(1);
                        for (const auto &pt : contour) shifted[0].emplace_back(pt.x - out.box.x, pt.y - out.box.y);
                        cv::drawContours(localMask, shifted, 0, cv::Scalar(255), cv::FILLED);
                        const IdentityScore refinedId = evaluateIdentity(bgr, out.box, &localMask, false);
                        if (refinedId.valid) {
                            out.score = std::clamp(refinedId.score, 0.0, 1.0);
                            found = true;
                        }
                    }
                    if (!found) {
                        out = Detection();
                        out.box = fullBox;
                        out.rotBox = cv::RotatedRect(
                            cv::Point2f(fullBox.x + fullBox.width * 0.5f, fullBox.y + fullBox.height * 0.5f),
                            cv::Size2f(float(fullBox.width), float(fullBox.height)), 0.0f);
                        out.center = cv::Point2d(fullBox.x + fullBox.width * 0.5, fullBox.y + fullBox.height * 0.5);
                        const double scaleFactor = std::sqrt(boxArea / refBoxArea);
                        out.area = m_reference.area * scaleFactor * scaleFactor;
                        out.aspectRatio = double(fullBox.height) / std::max(1, fullBox.width);
                        out.angle = m_reference.angle;
                        out.angleReliable = false;
                        out.score = std::clamp(id.score, 0.0, 1.0);
                        out.valid = true;
                        contour.clear();
                        found = true;
                    }
                }
            }
        }
        if (!found) {
            // 身份校验失败或 CSRT 丢目标：立刻废弃可能被污染的在线滤波器，转全图重找
            m_csrt.release();
            m_csrtActive = false;
        }
    }

    // 2. 丢失/大幅位移/遮挡恢复 或 跨视角初次搜索：全图颜色反投影 + 严格色度校验自动找回
    if (!found) {
        if (redetectTarget(bgr, out, contour, m_crossViewSearching)) {
            found = true;
            needReinitCsrt = true;
            if (m_crossViewSearching) {
                // 跨视角首次命中后，切换为本视角的本地参考框与融合光照签名，进入高速 CSRT 精跟
                m_crossViewSearching = false;
                m_reference = out;
                m_referenceContour = contour;
                m_referenceFromContour = !contour.empty();
                const cv::Rect clampedBox = clampRect(out.box, bgr.size());
                if (clampedBox.width >= 8 && clampedBox.height >= 8) {
                    cv::cvtColor(bgr(clampedBox), m_refGrayTemplate, cv::COLOR_BGR2GRAY);
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

    m_lostFrames = 0;
    m_trackedCenter = out.center;
    m_trackedArea = out.area;
    m_trackedScale = std::sqrt(out.area / std::max(1.0, m_reference.area));
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

    // 无人工选择且无需显示掩膜时直接跳过全帧边缘提取，降低三路 CPU 占用
    if (!m_hasPickedPoint && !m_hasTrackedTarget && !m_computeMask) {
        m_mask.release();
        return result;
    }

    // 1. 灰度化与高斯降噪
    cv::Mat gray, blurred;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.2);

    // 2. 自适应边缘提取
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

    // 5. 无人工选择时只生成掩膜，不自动选择最大的背景轮廓
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
