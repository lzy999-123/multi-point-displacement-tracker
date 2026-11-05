#include "channel.h"

#include <QElapsedTimer>

#include <cmath>

#include <opencv2/imgproc.hpp>

namespace {
// 界面最多积压 2 个未处理结果，超过就跳过显示（检测照常进行）
constexpr int kMaxPendingResults = 2;

QImage toQImage(const cv::Mat &bgr)
{
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    return QImage(rgb.data, rgb.cols, rgb.rows, int(rgb.step), QImage::Format_RGB888).copy();
}

QImage maskToQImage(const cv::Mat &mask)
{
    return QImage(mask.data, mask.cols, mask.rows, int(mask.step), QImage::Format_Grayscale8).copy();
}
} // namespace

// ---------------- ChannelWorker ----------------

ChannelWorker::ChannelWorker(std::shared_ptr<std::atomic<int>> pending)
    : m_pending(std::move(pending))
{
}

void ChannelWorker::init()
{
    m_stream = new MjpegStream(this);
    connect(m_stream, &MjpegStream::frameDecoded, this, &ChannelWorker::onFrame);
    connect(m_stream, &MjpegStream::stateChanged, this, &ChannelWorker::stateChanged);
}

void ChannelWorker::open(const QUrl &url)
{
    m_fpsWindowStart = -1;
    m_fpsFrames = 0;
    m_fps = 0.0;
    m_stream->open(url);
}

void ChannelWorker::close()
{
    if (m_stream)
        m_stream->close();
    m_lastFrame.release();
}

void ChannelWorker::setParams(const ColorParams &params)
{
    m_detector.setParams(params);
}

void ChannelWorker::setShowMask(bool show)
{
    m_showMask = show;
}

void ChannelWorker::pickColor(int x, int y)
{
    ColorParams params = m_detector.params();
    if (ColorBlobDetector::pickColor(m_lastFrame, x, y, &params)) {
        m_detector.setParams(params);
        emit colorPicked(params);
    }
}

void ChannelWorker::onFrame(const cv::Mat &bgr, quint64 seq, qint64 receivedMs)
{
    m_lastFrame = bgr;

    // 帧率：每 1 秒统计一次
    if (m_fpsWindowStart < 0)
        m_fpsWindowStart = receivedMs;
    ++m_fpsFrames;
    if (receivedMs - m_fpsWindowStart >= 1000) {
        m_fps = m_fpsFrames * 1000.0 / double(receivedMs - m_fpsWindowStart);
        m_fpsFrames = 0;
        m_fpsWindowStart = receivedMs;
    }

    QElapsedTimer timer;
    timer.start();
    ChannelResult result;
    result.detections = m_detector.detect(bgr);
    result.processMs = timer.nsecsElapsed() / 1e6;
    result.seq = seq;
    result.fps = m_fps;
    if (!result.detections.isEmpty()) {
        const cv::Point2d c = result.detections.first().center;
        result.hasTarget = true;
        result.offsetX = int(std::lround(c.x - bgr.cols / 2.0));
        result.offsetY = int(std::lround(c.y - bgr.rows / 2.0));
    }

    // 阶段 2 在这里把偏移发给云台（不受显示限流影响）

    if (m_pending->load() >= kMaxPendingResults)
        return;
    result.image = toQImage(bgr);
    if (m_showMask) {
        const cv::Mat mask = m_detector.lastMask();
        if (!mask.empty())
            result.mask = maskToQImage(mask);
    }
    m_pending->fetch_add(1);
    emit resultReady(result);
}

// ---------------- Channel ----------------

Channel::Channel(int id, QObject *parent)
    : QObject(parent), m_id(id), m_pending(std::make_shared<std::atomic<int>>(0))
{
    qRegisterMetaType<ChannelResult>();
    qRegisterMetaType<ColorParams>();
    qRegisterMetaType<MjpegStream::State>();

    m_worker = new ChannelWorker(m_pending);
    m_worker->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    connect(m_worker, &ChannelWorker::resultReady, this, &Channel::resultReady);
    connect(m_worker, &ChannelWorker::stateChanged, this, &Channel::stateChanged);
    connect(m_worker, &ChannelWorker::colorPicked, this, &Channel::colorPicked);
    m_thread.setObjectName(QStringLiteral("channel-%1").arg(id));
    m_thread.start();
    QMetaObject::invokeMethod(m_worker, &ChannelWorker::init, Qt::QueuedConnection);
}

Channel::~Channel()
{
    // 在工作线程内关闭网络连接，再退出线程
    QMetaObject::invokeMethod(m_worker, &ChannelWorker::close, Qt::BlockingQueuedConnection);
    m_thread.quit();
    m_thread.wait();
}

void Channel::open(const QUrl &url)
{
    m_url = url;
    m_open = true;
    m_pending->store(0);
    QMetaObject::invokeMethod(m_worker, [w = m_worker, url] { w->open(url); }, Qt::QueuedConnection);
}

void Channel::close()
{
    m_open = false;
    QMetaObject::invokeMethod(m_worker, &ChannelWorker::close, Qt::QueuedConnection);
}

void Channel::setParams(const ColorParams &params)
{
    QMetaObject::invokeMethod(m_worker, [w = m_worker, params] { w->setParams(params); },
                              Qt::QueuedConnection);
}

void Channel::setShowMask(bool show)
{
    QMetaObject::invokeMethod(m_worker, [w = m_worker, show] { w->setShowMask(show); },
                              Qt::QueuedConnection);
}

void Channel::pickColor(int x, int y)
{
    QMetaObject::invokeMethod(m_worker, [w = m_worker, x, y] { w->pickColor(x, y); },
                              Qt::QueuedConnection);
}

void Channel::acknowledgeResult()
{
    int v = m_pending->load();
    while (v > 0 && !m_pending->compare_exchange_weak(v, v - 1)) {
    }
}
