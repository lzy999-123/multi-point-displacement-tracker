#include "channel.h"

#include <QElapsedTimer>

#include <cmath>

#include <opencv2/imgproc.hpp>

namespace {
// 界面最多积压 2 个未处理结果，超过就跳过显示（检测照常进行）
constexpr int kMaxPendingResults = 2;

QImage toQImage(const cv::Mat &bgr)
{
    QImage image(bgr.cols, bgr.rows, QImage::Format_RGB888);
    cv::Mat rgb(bgr.rows, bgr.cols, CV_8UC3, image.bits(), size_t(image.bytesPerLine()));
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    return image;
}

cv::Mat enhanceForDisplay(const cv::Mat &bgr)
{
    if (bgr.empty() || bgr.type() != CV_8UC3) return bgr;
    cv::Mat blurred, sharp, contrasted, gray, gray3, out;
    cv::GaussianBlur(bgr, blurred, cv::Size(3, 3), 0.9);
    cv::addWeighted(bgr, 1.32, blurred, -0.32, 0.0, sharp);
    sharp.convertTo(contrasted, -1, 1.10, -10.0);
    cv::cvtColor(contrasted, gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(gray, gray3, cv::COLOR_GRAY2BGR);
    cv::addWeighted(contrasted, 1.22, gray3, -0.22, 0.0, out);
    return out;
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
    m_mjpegStream = new MjpegStream(this);
    connect(m_mjpegStream, &MjpegStream::frameDecoded, this, &ChannelWorker::onFrame);
    connect(m_mjpegStream, &MjpegStream::stateChanged, this, &ChannelWorker::stateChanged);

    m_udpStream = new UdpStream(this);
    connect(m_udpStream, &UdpStream::frameDecoded, this, &ChannelWorker::onFrame);
    connect(m_udpStream, &UdpStream::stateChanged, this, &ChannelWorker::stateChanged);
    connect(m_udpStream, &UdpStream::videoProfileApplied, this, &ChannelWorker::videoProfileApplied);

}

void ChannelWorker::open(const QUrl &url)
{
    close();
    // Picked/tracked coordinates belong to the previous sensor image size.
    m_autoDetector.resetTracking();
    m_fpsWindowStart = -1;
    m_fpsFrames = 0;
    m_fps = 0.0;
    if (url.scheme().toLower() == QStringLiteral("udp")) {
        if (m_udpStream) {
            emit activeStreamChanged(url);
            m_udpStream->open(url);
        }
    } else {
        if (m_mjpegStream) {
            emit activeStreamChanged(url);
            m_mjpegStream->open(url);
        }
    }
}

void ChannelWorker::close()
{
    if (m_udpStream)
        m_udpStream->close();
    if (m_mjpegStream)
        m_mjpegStream->close();
    m_lastFrame.release();
    m_lastDetection = Detection();
    m_lastFrameAtMs = -1;
    m_baseline.reset();
    m_debouncer.reset();
    m_autoDetector.resetTracking();
}

void ChannelWorker::setParams(const ColorParams &params)
{
    m_blobDetector.setParams(params);
}

void ChannelWorker::setAutoParams(const AutoContourParams &params)
{
    m_autoDetector.setParams(params);
}

void ChannelWorker::setDetectorMode(int mode)
{
    const auto selected = (mode == ColorBlob) ? ColorBlob : AutoContour;
    if (selected == m_mode) return;
    m_mode = selected;
    resetBaseline();
    m_autoDetector.resetTracking();
    m_lastDetection = Detection();
}

void ChannelWorker::calibrateBaseline()
{
    resetBaseline(); // 失败请求不能在后续帧中悄悄恢复先前的基准。
    const bool fresh = m_lastFrameAtMs >= 0 && monitorNowMs() - m_lastFrameAtMs <= kMonitorFreshnessMs;
    if (!fresh || !m_lastDetection.valid || m_lastDetection.area <= 0) {
        emit calibrationFinished(m_revision, false, fresh ? QStringLiteral("请先在画面中点击有效目标")
                                                          : QStringLiteral("没有近期新画面，不能锁定基准"));
        return;
    }
    m_baseline.calibrate(m_lastDetection, m_lastFrame);
    m_debouncer.reset();
    emit calibrationFinished(m_revision, true, QStringLiteral("已锁定当前选定目标的图像基准"));
}

void ChannelWorker::resetBaseline()
{
    m_baseline.reset();
    m_debouncer.reset();
}

void ChannelWorker::setThresholds(const MonitorThresholds &th)
{
    m_thresholds = th;
}

void ChannelWorker::setShowMask(bool show)
{
    m_showMask = show;
    m_autoDetector.setComputeMask(show);
}

void ChannelWorker::setImageEnhance(bool enable)
{
    m_imageEnhance = enable;
}

void ChannelWorker::pickTarget(int x, int y)
{
    resetBaseline();
    m_lastDetection = Detection();
    if (m_mode == AutoContour) {
        m_autoDetector.pickTarget(x, y);
    } else {
        pickColor(x, y);
    }
}

void ChannelWorker::selectTargetRoi(const QRect &roi)
{
    resetBaseline();
    m_lastDetection = Detection();
    if (m_mode == AutoContour) {
        m_autoDetector.selectRoi(cv::Rect(roi.x(), roi.y(), roi.width(), roi.height()));
    } else {
        pickColor(roi.center().x(), roi.center().y());
    }
}

void ChannelWorker::pickColor(int x, int y)
{
    resetBaseline();
    m_lastDetection = Detection();
    ColorParams params = m_blobDetector.params();
    if (ColorBlobDetector::pickColor(m_lastFrame, x, y, &params)) {
        m_blobDetector.setParams(params);
        emit colorPicked(params);
    }
}

void ChannelWorker::onFrame(const cv::Mat &bgr, quint64 seq, qint64 receivedMs)
{
    const qint64 observedAt = monitorNowMs();
    if (m_lastFrameAtMs < 0 || observedAt - m_lastFrameAtMs > kMonitorFreshnessMs)
        m_debouncer.reset(); // 断流期间不能累计“持续正常/异常”时间。
    m_lastFrameAtMs = observedAt;
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

    if (m_mode == AutoContour) {
        result.detections = m_autoDetector.detect(bgr);
    } else {
        result.detections = m_blobDetector.detect(bgr);
    }

    result.processMs = timer.nsecsElapsed() / 1e6;
    result.seq = seq;
    result.revision = m_revision;
    result.observedAtMs = observedAt;
    result.fps = m_fps;

    if (!result.detections.isEmpty()) {
        m_lastDetection = result.detections.first();
        const cv::Point2d c = m_lastDetection.center;
        result.hasTarget = true;
        result.offsetX = int(std::lround(c.x - bgr.cols / 2.0));
        result.offsetY = int(std::lround(c.y - bgr.rows / 2.0));
    } else {
        m_lastDetection = Detection();
        result.hasTarget = false;
        result.offsetX = 0;
        result.offsetY = 0;
    }

    // 藏品监测：比对基准计算偏差
    result.baselineCalibrated = m_baseline.isCalibrated;
    result.baselineDetection = m_baseline.baseDetection;
    result.anomaly = m_debouncer.update(m_baseline.evaluate(m_lastDetection, m_thresholds), observedAt);

    if (m_pending->load() >= kMaxPendingResults)
        return;

    result.image = toQImage(m_imageEnhance ? enhanceForDisplay(bgr) : bgr);
    if (m_showMask) {
        const cv::Mat mask = (m_mode == AutoContour) ? m_autoDetector.lastMask()
                                                     : m_blobDetector.lastMask();
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
    qRegisterMetaType<AutoContourParams>();
    qRegisterMetaType<MonitorThresholds>();
    qRegisterMetaType<AnomalyMetrics>();
    qRegisterMetaType<MjpegStream::State>();

    m_worker = new ChannelWorker(m_pending);
    m_worker->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    connect(m_worker, &ChannelWorker::resultReady, this, &Channel::resultReady);
    connect(m_worker, &ChannelWorker::stateChanged, this, &Channel::stateChanged);
    connect(m_worker, &ChannelWorker::colorPicked, this, &Channel::colorPicked);
    connect(m_worker, &ChannelWorker::activeStreamChanged, this, &Channel::activeStreamChanged);
    connect(m_worker, &ChannelWorker::videoProfileApplied, this, &Channel::videoProfileApplied);
    connect(m_worker, &ChannelWorker::calibrationFinished, this, &Channel::calibrationFinished);
    m_thread.setObjectName(QStringLiteral("channel-%1").arg(id));
    m_thread.start();
    QMetaObject::invokeMethod(m_worker, &ChannelWorker::init, Qt::QueuedConnection);
}

Channel::~Channel()
{
    // 等检测及收包关闭完成，再退出；强杀线程会遗留 UDP 收包线程和 socket。
    QMetaObject::invokeMethod(m_worker, &ChannelWorker::close, Qt::BlockingQueuedConnection);
    m_thread.quit();
    m_thread.wait();
}

void Channel::open(const QUrl &url)
{
    invalidateResults();
    m_url = url;
    m_open = true;
    m_pending->store(0);
    QMetaObject::invokeMethod(m_worker, [w = m_worker, url] {
        w->open(url);
    }, Qt::QueuedConnection);
}

void Channel::close()
{
    invalidateResults();
    m_open = false;
    QMetaObject::invokeMethod(m_worker, &ChannelWorker::close, Qt::QueuedConnection);
}

void Channel::setParams(const ColorParams &params)
{
    QMetaObject::invokeMethod(m_worker, [w = m_worker, params] { w->setParams(params); },
                              Qt::QueuedConnection);
}

void Channel::setAutoParams(const AutoContourParams &params)
{
    QMetaObject::invokeMethod(m_worker, [w = m_worker, params] { w->setAutoParams(params); },
                              Qt::QueuedConnection);
}

void Channel::setDetectorMode(int mode)
{
    invalidateResults();
    QMetaObject::invokeMethod(m_worker, [w = m_worker, mode] { w->setDetectorMode(mode); },
                              Qt::QueuedConnection);
}

void Channel::calibrateBaseline()
{
    invalidateResults();
    QMetaObject::invokeMethod(m_worker, &ChannelWorker::calibrateBaseline, Qt::QueuedConnection);
}

void Channel::resetBaseline()
{
    invalidateResults();
    QMetaObject::invokeMethod(m_worker, &ChannelWorker::resetBaseline, Qt::QueuedConnection);
}

void Channel::setThresholds(const MonitorThresholds &th)
{
    QMetaObject::invokeMethod(m_worker, [w = m_worker, th] { w->setThresholds(th); },
                              Qt::QueuedConnection);
}

void Channel::setShowMask(bool show)
{
    QMetaObject::invokeMethod(m_worker, [w = m_worker, show] { w->setShowMask(show); },
                              Qt::QueuedConnection);
}

void Channel::setImageEnhance(bool enable)
{
    QMetaObject::invokeMethod(m_worker, [w = m_worker, enable] { w->setImageEnhance(enable); },
                              Qt::QueuedConnection);
}

void Channel::pickTarget(int x, int y)
{
    invalidateResults();
    QMetaObject::invokeMethod(m_worker, [w = m_worker, x, y] { w->pickTarget(x, y); },
                              Qt::QueuedConnection);
}

void Channel::selectTargetRoi(const QRect &roi)
{
    invalidateResults();
    QMetaObject::invokeMethod(m_worker, [w = m_worker, roi] { w->selectTargetRoi(roi); },
                              Qt::QueuedConnection);
}

void Channel::pickColor(int x, int y)
{
    invalidateResults();
    QMetaObject::invokeMethod(m_worker, [w = m_worker, x, y] { w->pickColor(x, y); },
                              Qt::QueuedConnection);
}

void Channel::acknowledgeResult()
{
    int v = m_pending->load();
    while (v > 0 && !m_pending->compare_exchange_weak(v, v - 1)) {
    }
}

void Channel::invalidateResults()
{
    const quint64 revision = ++m_revision;
    QMetaObject::invokeMethod(m_worker, [w = m_worker, revision] { w->setRevision(revision); },
                              Qt::QueuedConnection);
}
