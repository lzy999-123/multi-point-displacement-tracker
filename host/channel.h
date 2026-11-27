#pragma once

#include <QImage>
#include <QObject>
#include <QPair>
#include <QPointF>
#include <QThread>
#include <QUrl>
#include <QVector>

#include <atomic>
#include <memory>

#include "artifactmonitor.h"
#include "autocontourdetector.h"
#include "colorblobdetector.h"
#include "mjpegstream.h"
#include "udpstream.h"

// 一路处理结果，从工作线程发到界面线程。
struct ChannelResult {
    QImage image;               // 原始画面（RGB888）
    QImage mask;                // 检测掩膜（仅开启显示掩膜时有效，Grayscale8）
    QList<Detection> detections;
    bool hasTarget = false;
    bool trackingActive = false;       // 是否已人工选定目标（含丢失搜索期）
    int offsetX = 0, offsetY = 0;      // 目标中心减画面中心，右、下为正
    QPointF targetCenter;              // 目标当前图像像素坐标
    double deltaX = 0.0, deltaY = 0.0; // 相对跟踪起点的位移 (px)
    double displacement = 0.0;         // 相对跟踪起点的欧氏位移 (px)
    double speedPxPerSec = 0.0;        // 瞬时移动速度 (px/s)
    double scaleRatio = 1.0;           // 相对初始目标面积的尺度比
    QVector<QPointF> trajectory;       // 最近运动轨迹点（图像坐标）
    quint64 seq = 0;
    quint64 revision = 0;       // 重连/重选/重标定前的排队结果不能复活旧状态
    qint64 observedAtMs = -1;   // 全局单调时钟，采样进入检测的时刻
    double fps = 0.0;           // 解码帧率（最近 1 秒）
    double processMs = 0.0;     // 检测耗时

    // 藏品监测与基准状态
    bool baselineCalibrated = false;
    Detection baselineDetection;
    AnomalyMetrics anomaly;
    TargetStatus targetStatus;  // 目标实时综合状态（是否移动过 / 变大变小 / 是否倾倒，锁定即输出）
};
Q_DECLARE_METATYPE(ChannelResult)

// 工作线程中的对象：取流 + 检测 + 异动评估。只通过 Channel 的排队调用访问。
class ChannelWorker : public QObject
{
    Q_OBJECT
public:
    enum DetectorMode {
        AutoContour = 0,   // 免标定轮廓提取（默认，无需调色）
        ColorBlob = 1      // 颜色色块检测（向后兼容）
    };

    explicit ChannelWorker(std::shared_ptr<std::atomic<int>> pending);

public slots:
    void init();
    void setRevision(quint64 revision) { m_revision = revision; }
    void open(const QUrl &url);
    void close();
    void setParams(const ColorParams &params);
    void setAutoParams(const AutoContourParams &params);
    void setDetectorMode(int mode);
    void calibrateBaseline();
    void resetBaseline();
    void setThresholds(const MonitorThresholds &th);
    void setShowMask(bool show);
    void setImageEnhance(bool enable);
    void pickTarget(int x, int y);
    void selectTargetRoi(const QRect &roi);
    void syncTargetSignature(const TargetSignature &sig);
    void clearTarget();
    void resetTrajectoryOrigin();
    void pickColor(int x, int y);

signals:
    void resultReady(const ChannelResult &result);
    void stateChanged(MjpegStream::State state, const QString &detail);
    void colorPicked(const ColorParams &params);
    void activeStreamChanged(const QUrl &url);
    void videoProfileApplied(int profile);
    void calibrationFinished(quint64 revision, bool success, const QString &reason);
    void targetSignatureExtracted(quint64 revision, const TargetSignature &sig);

private:
    void onFrame(const cv::Mat &bgr, quint64 seq, qint64 receivedMs);

    MjpegStream *m_mjpegStream = nullptr;
    UdpStream *m_udpStream = nullptr;
    AutoContourDetector m_autoDetector;
    ColorBlobDetector m_blobDetector;
    DetectorMode m_mode = ColorBlob;

    BaselineState m_baseline;
    MonitorThresholds m_thresholds;
    AnomalyDebouncer m_debouncer;
    TargetStateTracker m_targetTracker;
    quint64 m_revision = 0;
    qint64 m_lastFrameAtMs = -1;

    std::shared_ptr<std::atomic<int>> m_pending;
    cv::Mat m_lastFrame;
    Detection m_lastDetection;
    bool m_showMask = false;
    bool m_imageEnhance = false;
    bool m_pendingSignatureBroadcast = false;
    bool m_hasTrackOrigin = false;
    QPointF m_trackOrigin;
    double m_trackOriginArea = 0.0;
    bool m_hasPrevTrack = false;
    QPointF m_prevTrackCenter;
    qint64 m_prevTrackMs = -1;
    double m_smoothSpeed = 0.0;
    QVector<QPair<QPointF, qint64>> m_motionWindow;
    cv::Mat m_motionRefGray;
    qint64 m_motionRefMs = -1;
    QVector<QPair<QPointF, qint64>> m_timedTrajectory;
    bool m_hasStillAnchor = false;
    QPointF m_stillAnchor;
    qint64 m_fpsWindowStart = -1;
    int m_fpsFrames = 0;
    double m_fps = 0.0;
};

// 界面线程使用的一路通道，内部持有工作线程。
class Channel : public QObject
{
    Q_OBJECT
public:
    explicit Channel(int id, QObject *parent = nullptr);
    ~Channel() override;

    int id() const { return m_id; }
    QUrl url() const { return m_url; }
    bool isOpen() const { return m_open; }
    quint64 revision() const { return m_revision; }

    void open(const QUrl &url);
    void close();
    void setParams(const ColorParams &params);
    void setAutoParams(const AutoContourParams &params);
    void setDetectorMode(int mode);
    void calibrateBaseline();
    void resetBaseline();
    void setThresholds(const MonitorThresholds &th);
    void setShowMask(bool show);
    void setImageEnhance(bool enable);
    void pickTarget(int x, int y);
    void selectTargetRoi(const QRect &roi);
    void syncTargetSignature(const TargetSignature &sig);
    void clearTarget();
    void resetTrajectoryOrigin();
    void pickColor(int x, int y);  // 图像坐标

    // 界面处理完一个结果后调用，允许工作线程发送下一个（防止界面卡顿时结果堆积）
    void acknowledgeResult();

signals:
    void resultReady(const ChannelResult &result);
    void stateChanged(MjpegStream::State state, const QString &detail);
    void colorPicked(const ColorParams &params);
    void activeStreamChanged(const QUrl &url);
    void videoProfileApplied(int profile);
    void calibrationFinished(quint64 revision, bool success, const QString &reason);
    void targetSignatureExtracted(quint64 revision, const TargetSignature &sig);

private:
    void invalidateResults();
    int m_id;
    QUrl m_url;
    bool m_open = false;
    quint64 m_revision = 0;
    QThread m_thread;
    ChannelWorker *m_worker = nullptr;
    std::shared_ptr<std::atomic<int>> m_pending;
};
