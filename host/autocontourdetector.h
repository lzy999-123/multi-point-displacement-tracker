#pragma once

#include <opencv2/core.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/video/tracking.hpp>
#include "targetdetector.h"

// 特征跟踪器参数
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

// 跨检测点共享的目标外观特征签名（视角/位置无关，用于单路框选、三路自动联动锁定）
struct TargetSignature {
    bool valid = false;
    bool isChromatic = false;
    cv::Mat hsHist;           // 24x12 归一化 HSV 色相-饱和度直方图
    cv::Mat labHist;          // 8x8x8 归一化 CIE Lab 直方图
    cv::Vec3d coreLab{0.0, 128.0, 128.0};
    double chromaticFrac = 0.0;
    cv::Mat grayTemplate;     // 紧致无背景灰阶模板（用于非彩色目标跨视角匹配）
    cv::Size refBoxSize;      // 源视角目标外接框尺寸
    double refArea = 0.0;     // 源视角目标面积
    cv::Size sourceFrameSize; // 源画面尺寸
    double aspectRatio = 1.0; // 目标高宽比
};
Q_DECLARE_METATYPE(TargetSignature)

// 多检测点特征跟踪器：
// 1. 拖拽框选目标（selectRoi）：CSRT 多通道相关滤波（HOG 纹理 + Color-Names 颜色 + RGB）
//    + 核心前景 HSV/Lab 色度身份门控（严防黄色瓶子串跳到其他颜色物体）
//    + 目标丢失/遮挡/大幅移动后的颜色反投影全图自动重找 + 2D 卡尔曼运动估计；
// 2. 跨视角特征联动（exportSignature / adoptSignature）：单路框选后提取视角无关色度+外观签名，
//    自动在其余摄像头全图搜出同目标并切换为本视角独立 CSRT 精跟；
// 3. 单击轮廓内部（pickTarget）：保留边缘轮廓点选关联以兼容传统轮廓模式。
// 未人工选择时不自动接管背景；校验不通过或目标移出视野返回空（不乱吸附其他物体）。
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
    // 拖拽框选目标区域（CSRT 多特征跟踪）
    void selectRoi(const cv::Rect &roi);
    // 导出当前已锁定目标的外观签名，供其余检测点跨视角联动锁定
    TargetSignature exportSignature(const cv::Mat &bgr) const;
    // 接收来自另一检测点的外观签名，开启全图搜索并在命中后切入本视角 CSRT 精跟
    void adoptSignature(const TargetSignature &sig);
    void resetTracking();
    bool hasActiveSelection() const { return m_roiMode || m_hasPendingRoi || m_hasTrackedTarget || m_hasPickedPoint; }

private:
    struct IdentityScore {
        bool valid = false;
        double score = 0.0;
        double hsSim = 0.0;
        double labSim = 0.0;
        double abDist = 999.0;
        double chromaticFrac = 0.0;
    };

    QList<Detection> initAndDetectRoi(const cv::Mat &bgr);
    QList<Detection> trackRoi(const cv::Mat &bgr);
    void initCsrtTracker(const cv::Mat &bgr, const cv::Rect &fullBox);
    void initKalmanAt(const cv::Point2d &center);
    IdentityScore evaluateIdentity(const cv::Mat &bgr, const cv::Rect &box,
                                   const cv::Mat *fgMask = nullptr, bool strictForRedetect = false,
                                   bool crossView = false) const;
    bool redetectTarget(const cv::Mat &bgr, Detection &outDet, std::vector<cv::Point> &outContour,
                        bool crossView = false);
    bool refineContourInBox(const cv::Mat &bgr, const cv::Rect &box,
                            Detection &outDet, std::vector<cv::Point> &outContour) const;

    AutoContourParams m_params;
    cv::Mat m_mask;
    bool m_computeMask = false;

    bool m_hasPickedPoint = false;
    cv::Point2d m_pickedPoint;

    bool m_hasPendingRoi = false;
    cv::Rect m_pendingRoi;
    bool m_roiMode = false;
    bool m_crossViewSearching = false;
    cv::Size m_sourceFrameSize;
    bool m_referenceFromContour = false;

    // CSRT 在线多特征跟踪器（在降采样图上运行以兼顾三路实时性与高精度）
    cv::Ptr<cv::TrackerCSRT> m_csrt;
    bool m_csrtActive = false;
    double m_csrtScale = 1.0;
    int m_lostFrames = 0;

    // 目标初始外观签名（永不被误匹配污染，专用于防串色门控与全图重找）
    cv::Mat m_refGrayTemplate;     // 紧致无背景灰阶模板
    cv::Mat m_refHsHist;           // 2D HSV 色相-饱和度直方图（仅统计高饱和前景像素）
    cv::Mat m_crossHsHist;         // 跨视角平滑 2D HSV 直方图（吸收多机白平衡微小差异）
    cv::Mat m_refLabHist;          // 3D CIE Lab 直方图（仅统计核心前景像素）
    cv::Vec3d m_refCoreLab{0.0, 128.0, 128.0};
    double m_refChromaticFrac = 0.0;
    bool m_isChromatic = false;
    cv::Vec3d m_bgLabMean{0.0, 0.0, 0.0};
    double m_bgLabSpread = 22.0;
    bool m_bgRingReliable = false;
    double m_trackedScale = 1.0;

    // 2D 卡尔曼滤波器（状态 [x, y, vx, vy]）
    cv::KalmanFilter m_kalman;
    bool m_kalmanReady = false;

    bool m_hasTrackedTarget = false;
    cv::Point2d m_trackedCenter;
    double m_trackedArea = 0.0;
    Detection m_reference;
    std::vector<cv::Point> m_referenceContour;
};
