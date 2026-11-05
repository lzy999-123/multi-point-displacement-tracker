#include "mjpegstream.h"

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>

#include <opencv2/imgcodecs.hpp>

namespace {
constexpr int kReconnectDelayMs = 1000;
constexpr int kNoFrameTimeoutMs = 3000;
} // namespace

MjpegStream::MjpegStream(QObject *parent)
    : QObject(parent), m_nam(new QNetworkAccessManager(this))
{
    qRegisterMetaType<cv::Mat>();
    m_clock.start();
    m_reconnectTimer.setSingleShot(true);
    m_reconnectTimer.setInterval(kReconnectDelayMs);
    connect(&m_reconnectTimer, &QTimer::timeout, this, &MjpegStream::startRequest);

    m_watchdog.setSingleShot(true);
    m_watchdog.setInterval(kNoFrameTimeoutMs);
    connect(&m_watchdog, &QTimer::timeout, this, [this] {
        scheduleReconnect(tr("%1 秒没有收到画面").arg(kNoFrameTimeoutMs / 1000));
    });
}

MjpegStream::~MjpegStream()
{
    close();
}

void MjpegStream::open(const QUrl &url)
{
    close();
    m_url = url;
    m_wanted = true;
    startRequest();
}

void MjpegStream::close()
{
    m_wanted = false;
    m_reconnectTimer.stop();
    m_watchdog.stop();
    if (m_reply) {
        QNetworkReply *reply = m_reply;
        m_reply = nullptr;
        reply->disconnect(this);
        reply->abort();
        reply->deleteLater();
    }
    m_parser.reset();
    setState(State::Idle);
}

void MjpegStream::startRequest()
{
    if (!m_wanted)
        return;
    m_parser.reset();
    m_seq = 0;

    QNetworkRequest request(m_url);
    request.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::AlwaysNetwork);
    request.setAttribute(QNetworkRequest::CacheSaveControlAttribute, false);
    request.setTransferTimeout(kNoFrameTimeoutMs);

    m_reply = m_nam->get(request);
    m_reply->setReadBufferSize(0);
    connect(m_reply, &QNetworkReply::readyRead, this, &MjpegStream::onReadyRead);
    connect(m_reply, &QNetworkReply::finished, this, &MjpegStream::onFinished);
    m_watchdog.start();
    setState(State::Connecting, m_url.toString());
}

void MjpegStream::onReadyRead()
{
    if (!m_reply)
        return;
    const QList<QByteArray> frames = m_parser.feed(m_reply->readAll());
    if (frames.isEmpty())
        return;

    m_seq += frames.size();
    const qint64 receivedMs = m_clock.elapsed();
    const QByteArray &latest = frames.last();
    const cv::Mat raw(1, latest.size(), CV_8UC1, const_cast<char *>(latest.constData()));
    cv::Mat bgr = cv::imdecode(raw, cv::IMREAD_COLOR);
    if (bgr.empty())
        return;  // 损坏的 JPEG：等下一帧，看门狗不复位

    m_watchdog.start();
    if (m_state != State::Streaming)
        setState(State::Streaming, tr("%1×%2").arg(bgr.cols).arg(bgr.rows));
    emit frameDecoded(bgr, m_seq, receivedMs);
}

void MjpegStream::onFinished()
{
    if (!m_reply)
        return;
    const QString reason = m_reply->error() == QNetworkReply::NoError
                               ? tr("服务器关闭了连接")
                               : m_reply->errorString();
    scheduleReconnect(reason);
}

void MjpegStream::scheduleReconnect(const QString &reason)
{
    m_watchdog.stop();
    if (m_reply) {
        QNetworkReply *reply = m_reply;
        m_reply = nullptr;
        reply->disconnect(this);
        reply->abort();
        reply->deleteLater();
    }
    if (!m_wanted)
        return;
    setState(State::Reconnecting, reason);
    m_reconnectTimer.start();
}

void MjpegStream::setState(State state, const QString &detail)
{
    if (state == m_state && detail.isEmpty())
        return;
    m_state = state;
    emit stateChanged(state, detail);
}
