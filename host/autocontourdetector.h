#pragma once

#include <QString>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
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

// 跨检测点共享的目标外观与语义签名（视角/位置无关，用于单路框选、三路自动联动锁定）
struct TargetSignature {
    bool valid = false;
    bool isChromatic = false;
    int yoloClassId = -1;     // YOLO26n 识别到的 COCO 语义类别（-1 表示非 YOLO 目标，走纯视觉多特征）
    QString yoloClassName;    // 语义类别中文名（如“瓶子”、“杯子”）
    cv::Mat hsHist;           // 24x12 归一化 HSV 色相-饱和度直方图
    cv::Mat labHist;          // 8x8x8 归一化 CIE Lab 直方图
    cv::Vec3d coreLab{0.0, 128.0, 128.0};
    double chromaticFrac = 0.0;
    cv::Mat grayTemplate;     // 紧致无背景灰阶模板（用于非彩色/纹理目标校验与跨视角匹配）
    std::vector<cv::KeyPoint> orbKeypoints; // ORB 局部关键点（旋转无关纹理校验）
    cv::Mat orbDescriptors;   // ORB 描述子
    cv::Size orbPatchSize;    // ORB 提取时的块尺寸
    cv::Size refBoxSize;      // 源视角目标外接框尺寸
    double refArea = 0.0;     // 源视角目标面积
    cv::Size sourceFrameSize; // 源画面尺寸
    double aspectRatio = 1.0; // 目标高宽比
};
Q_DECLARE_METATYPE(TargetSignature)

// 多检测点混合特征跟踪器（融合 untitled7 的抗干扰与稳框机制）：
// 1. 语义+多特征锁定（selectRoi）：初始化时自动调用 YOLO26n 识别目标语义类别并锚定完整外接框，
//    同时提取 HSV/Lab 色度直方图 + 灰阶模板 + ORB 关键点描述子，由 CSRT 在线高速跟踪；
// 2. 稳框防缩防抖：借鉴 untitled7 的尺寸死区 + 非对称指数平滑（防反光/标签局部截断导致锁定框误缩小），
//    并引入不可靠帧短时 HOLD 缓冲与周期性 YOLO 完整框校准；
// 3. 强抗干扰跨视角联动（exportSignature / adoptSignature）：
//    - YOLO 已识别类别（如瓶子）时，其余摄像头只接受“同类 YOLO 检测框（含 90° 倾倒补检）+ 颜色外观校验”，
//      彻底杜绝柜子边角、黄色小标签、纸箱等背景干扰物被误锁；
//    - 非 YOLO 目标使用 ORB + 颜色 + 旋转无关形状相似度 + 模板的多线索联合门控；
//    - 跨视角/丢失重捕均经过多帧空间一致确认后才切入锁定，消除单帧闪锁与输出剧烈跳变。
class AutoContourDetector : public TargetDetector
{
public:
    AutoContourDetector() = default;

    void setParams(const AutoContourParams &params) { m_params = params; }
    const AutoContourParams &params() const { return m_params; }
    void setComputeMask(bool compute) { m_computeMask = compute; }

    // 实时流稳定性参数（单元测试默认保持单帧即时响应，实时通道开启多帧确认/防抖/限频）
    void setCrossViewConfirmFrames(int frames) { m_crossViewConfirmFrames = std::max(1, frames); }
    void setLostRecoveryConfirmFrames(int frames) { m_lostRecoveryConfirmFrames = std::max(1, frames); }
    void setRecoveryThrottleMs(qint64 ms) { m_recoveryThrottleMs = std::max<qint64>(0, ms); }
    void setBoxSmoothingEnabled(bool enabled) { m_boxSmoothingEnabled = enabled; }
    void setYoloEnabled(bool enabled) { m_yoloEnabled = enabled; }

    int lockedYoloClassId() const { return m_yoloClassId; }
    QString lockedYoloClassName() const { return m_yoloClassName; }

    QList<Detection> detect(const cv::Mat &bgr) override;
    cv::Mat lastMask() const override { return m_mask; }

    // 点击画面坐标选择特定目标轮廓
    void pickTarget(int x, int y);
    // 拖拽框选目标区域（YOLO 语义锚定 + CSRT 多特征跟踪）
    void selectRoi(const cv::Rect &roi);
    // 导出当前已锁定目标的外观与语义签名，供其余检测点跨视角联动锁定
    TargetSignature exportSignature(const cv::Mat &bgr) const;
    // 接收来自另一检测点的签名，开启全图搜索并在连续确认命中后切入本视角 CSRT 精跟
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
    void buildOrbReference(const cv::Mat &bgr, const cv::Rect &box);
    IdentityScore evaluateIdentity(const cv::Mat &bgr, const cv::Rect &box,
                                   const cv::Mat *fgMask = nullptr, bool strictForRedetect = false,
                                   bool crossView = false) const;
    bool redetectTarget(const cv::Mat &bgr, Detection &outDet, std::vector<cv::Point> &outContour,
                        bool crossView = false);
    bool redetectByYolo(const cv::Mat &bgr, Detection &outDet, std::vector<cv::Point> &outContour,
                        bool crossView);
    bool redetectByOrb(const cv::Mat &bgr, Detection &outDet, std::vector<cv::Point> &outContour,
                       bool crossView) const;
    bool confirmRecoveryCandidate(const cv::Rect &box, int requiredFrames);
    void resetRecoveryCandidate();
    cv::Rect smoothOutputBox(const cv::Rect &detectedBox, const cv::Size &frameSize, bool snap);
    double smoothOutputAngle(double angleDeg, bool snap);
    bool refineContourInBox(const cv::Mat &bgr, const cv::Rect &box,
                            Detection &outDet, std::vector<cv::Point> &outContour) const;

    AutoContourParams m_params;
    cv::Mat m_mask;
    bool m_computeMask = false;

    int m_crossViewConfirmFrames = 1;
    int m_lostRecoveryConfirmFrames = 1;
    qint64 m_recoveryThrottleMs = 0;
    bool m_boxSmoothingEnabled = false;
    bool m_yoloEnabled = true;

    bool m_hasPickedPoint = false;
    cv::Point2d m_pickedPoint;

    bool m_hasPendingRoi = false;
    cv::Rect m_pendingRoi;
    bool m_roiMode = false;
    bool m_crossViewSearching = false;
    cv::Size m_sourceFrameSize;
    bool m_referenceFromContour = false;

    // YOLO 语义类别锁与周期性完整外接框校准
    int m_yoloClassId = -1;
    QString m_yoloClassName;
    bool m_yoloBoxAnchored = false;
    qint64 m_lastYoloAnchorMs = -1;
    qint64 m_lastRecoveryMs = -1;

    // CSRT 在线多特征跟踪器（在降采样图上运行以兼顾三路实时性与高精度）
    cv::Ptr<cv::TrackerCSRT> m_csrt;
    bool m_csrtActive = false;
    double m_csrtScale = 1.0;
    int m_lostFrames = 0;
    int m_framesSinceCsrtReinit = 0;
    int m_unreliableCount = 0;

    // 稳框平滑状态与短时 HOLD 上一可靠检测结果
    bool m_hasSmoothedBox = false;
    cv::Rect2d m_smoothedBox;
    bool m_hasSmoothedAngle = false;
    double m_smoothedAngle = 0.0;
    Detection m_lastReliableDet;
    std::vector<cv::Point> m_lastReliableContour;

    // 重捕/跨视角连续帧确认缓存（借鉴 untitled7::confirmFeatureRecoveryCandidate）
    cv::Rect m_recoveryCandBox;
    int m_recoveryCandHits = 0;

    // 目标初始外观签名（永不被误匹配污染，专用于防串色门控与全图重找）
    cv::Mat m_refGrayTemplate;     // 紧致无背景灰阶模板
    std::vector<cv::KeyPoint> m_refOrbKeypoints;
    cv::Mat m_refOrbDescriptors;
    cv::Size m_refOrbPatchSize;
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
