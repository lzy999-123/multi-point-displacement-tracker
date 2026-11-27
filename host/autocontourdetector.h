#pragma once

#include <opencv2/core.hpp>
#include "targetdetector.h"

// 免标定轮廓检测器参数
struct AutoContourParams {
    int minArea = 250;            // 最小目标面积（过滤细小噪点）
    int maxTargets = 1;           // 最多返回的目标数量
    int morphKernelSize = 7;      // 边缘闭运算核大小（闭合物体边缘断线）
    bool autoThreshold = true;    // 使用自适应中值门限
    int cannyLow = 35;            // 固定模式下的 Canny 低阈值
    int cannyHigh = 100;          // 固定模式下的 Canny 高阈值

    bool operator==(const AutoContourParams &o) const {
        return minArea == o.minArea && maxTargets == o.maxTargets
            && morphKernelSize == o.morphKernelSize && autoThreshold == o.autoThreshold
            && cannyLow == o.cannyLow && cannyHigh == o.cannyHigh;
    }
    bool operator!=(const AutoContourParams &o) const { return !(*this == o); }
};
Q_DECLARE_METATYPE(AutoContourParams)

// 固定机位的人工选目标监测器。
// 支持两种人工锁定方式：
// 1. 拖拽框选目标区域（selectRoi）：多尺度模板匹配 + 局部 Lab 背景对比轮廓细化，
//    适合真实桌面/展柜场景下跟踪目标的移动、变大、变小、倾斜与消失；
// 2. 单击轮廓内部（pickTarget）：基于边缘轮廓的位置/面积/形状硬门限关联。
// 未人工选择时不自动接管背景；匹配失败返回空（目标不可见）。
class AutoContourDetector : public TargetDetector
{
public:
    AutoContourDetector() = default;

    void setParams(const AutoContourParams &params) { m_params = params; }
    const AutoContourParams &params() const { return m_params; }
    void setComputeMask(bool compute) { m_computeMask = compute; }

    QList<Detection> detect(const cv::Mat &bgr) override;
    cv::Mat lastMask() const override { return m_mask; }

    // 点击画面坐标选择特定目标轮廓
    void pickTarget(int x, int y);
    // 拖拽框选目标区域（固定机位展品跟踪）
    void selectRoi(const cv::Rect &roi);
    void resetTracking();

private:
    QList<Detection> initAndDetectRoi(const cv::Mat &bgr);
    QList<Detection> trackRoi(const cv::Mat &bgr);

    AutoContourParams m_params;
    cv::Mat m_mask;
    bool m_computeMask = false;

    bool m_hasPickedPoint = false;
    cv::Point2d m_pickedPoint;

    bool m_hasPendingRoi = false;
    cv::Rect m_pendingRoi;
    bool m_roiMode = false;
    bool m_referenceFromContour = false;
    cv::Mat m_refTemplate;
    cv::Mat m_refHist;
    cv::Vec3d m_bgLabMean{0.0, 0.0, 0.0};
    double m_bgLabSpread = 22.0;
    bool m_bgRingReliable = false;
    double m_trackedScale = 1.0;

    bool m_hasTrackedTarget = false;
    cv::Point2d m_trackedCenter;
    double m_trackedArea = 0.0;
    Detection m_reference;
    std::vector<cv::Point> m_referenceContour;
};
