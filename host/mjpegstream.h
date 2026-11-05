#pragma once

#include <QElapsedTimer>
#include <QObject>
#include <QPointer>
#include <QTimer>
#include <QUrl>

#include <opencv2/core.hpp>

#include "mjpegparser.h"

Q_DECLARE_METATYPE(cv::Mat)

class QNetworkAccessManager;
class QNetworkReply;

// 从 ESP32-CAM 的 HTTP MJPEG 地址取流并解码。
// 每次收到一批数据只解码最新的一张 JPEG，处理不过来时自动丢旧帧，不累积延迟。
// 断线或 3 秒没有新帧时自动重连。对象必须在其所属线程内使用。
class MjpegStream : public QObject
{
    Q_OBJECT
public:
    enum class State { Idle, Connecting, Streaming, Reconnecting };
    Q_ENUM(State)

    explicit MjpegStream(QObject *parent = nullptr);
    ~MjpegStream() override;

    void open(const QUrl &url);
    void close();
    State state() const { return m_state; }

signals:
    // seq 为本连接内收到的帧序号（含被跳过的帧），receivedMs 为接收时刻（单调时钟）
    void frameDecoded(const cv::Mat &bgr, quint64 seq, qint64 receivedMs);
    void stateChanged(MjpegStream::State state, const QString &detail);

private:
    void startRequest();
    void onReadyRead();
    void onFinished();
    void scheduleReconnect(const QString &reason);
    void setState(State state, const QString &detail = {});

    QNetworkAccessManager *m_nam = nullptr;
    QPointer<QNetworkReply> m_reply;
    MjpegParser m_parser;
    QUrl m_url;
    bool m_wanted = false;
    State m_state = State::Idle;
    quint64 m_seq = 0;
    QTimer m_reconnectTimer;
    QTimer m_watchdog;
    QElapsedTimer m_clock;
};
