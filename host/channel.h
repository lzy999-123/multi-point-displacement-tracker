#pragma once

#include <QImage>
#include <QObject>
#include <QThread>
#include <QUrl>

#include <atomic>
#include <memory>

#include "colorblobdetector.h"
#include "mjpegstream.h"
#include "udpstream.h"

// 一路处理结果，从工作线程发到界面线程。
struct ChannelResult {
    QImage image;               // 原始画面（RGB888）
    QImage mask;                // 检测掩膜（仅开启显示掩膜时有效，Grayscale8）
    QList<Detection> detections;
    bool hasTarget = false;
    int offsetX = 0, offsetY = 0;  // 目标中心减画面中心，右、下为正
    quint64 seq = 0;
    double fps = 0.0;           // 解码帧率（最近 1 秒）
    double processMs = 0.0;     // 检测耗时
};
Q_DECLARE_METATYPE(ChannelResult)

// 工作线程中的对象：取流 + 检测。只通过 Channel 的排队调用访问。
class ChannelWorker : public QObject
{
    Q_OBJECT
public:
    explicit ChannelWorker(std::shared_ptr<std::atomic<int>> pending);

public slots:
    void init();
    void open(const QUrl &url);
    void close();
    void setParams(const ColorParams &params);
    void setShowMask(bool show);
    void pickColor(int x, int y);

signals:
    void resultReady(const ChannelResult &result);
    void stateChanged(MjpegStream::State state, const QString &detail);
    void colorPicked(const ColorParams &params);

private:
    void onFrame(const cv::Mat &bgr, quint64 seq, qint64 receivedMs);

    MjpegStream *m_mjpegStream = nullptr;
    UdpStream *m_udpStream = nullptr;
    ColorBlobDetector m_detector;
    std::shared_ptr<std::atomic<int>> m_pending;
    cv::Mat m_lastFrame;
    bool m_showMask = false;
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

    void open(const QUrl &url);
    void close();
    void setParams(const ColorParams &params);
    void setShowMask(bool show);
    void pickColor(int x, int y);  // 图像坐标

    // 界面处理完一个结果后调用，允许工作线程发送下一个（防止界面卡顿时结果堆积）
    void acknowledgeResult();

signals:
    void resultReady(const ChannelResult &result);
    void stateChanged(MjpegStream::State state, const QString &detail);
    void colorPicked(const ColorParams &params);

private:
    int m_id;
    QUrl m_url;
    bool m_open = false;
    QThread m_thread;
    ChannelWorker *m_worker = nullptr;
    std::shared_ptr<std::atomic<int>> m_pending;
};
