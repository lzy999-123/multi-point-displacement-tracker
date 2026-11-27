#pragma once

#include <QByteArray>
#include <QObject>
#include <QThread>
#include <QTimer>
#include <QUrl>

#include <opencv2/core.hpp>

#include "mjpegstream.h"

class UdpFrameReceiver;

#pragma pack(push, 1)
struct VidPacketHeader {
    quint32 magic;         // 0x31444956 ("VID1")
    quint8  device_id;     // 1~3
    quint8  reserved;      // 0
    quint16 frame_seq;     // 0~65535
    quint8  chunk_index;   // 0 .. chunk_count-1
    quint8  chunk_count;   // 总分片数
    quint16 payload_size;  // 该包负载字节数
    quint32 frame_size;    // 完整 JPEG 字节数
};
#pragma pack(pop)

// 从 ESP32-CAM 接收轻量 UDP 视频分片（VID1 协议）并组帧解码。
// 专用线程收包并确认完整帧；所属线程只解码最新 JPEG，不积压检测任务。
class UdpStream : public QObject
{
    Q_OBJECT
public:
    explicit UdpStream(QObject *parent = nullptr);
    ~UdpStream() override;

    void open(const QUrl &url);
    void close();
    MjpegStream::State state() const { return m_state; }

signals:
    void frameDecoded(const cv::Mat &bgr, quint64 seq, qint64 receivedMs);
    void stateChanged(MjpegStream::State state, const QString &detail);
    void videoProfileApplied(int profile);

private:
    void decodePending();
    void onTimeout();
    void setState(MjpegStream::State state, const QString &detail = {});

    QThread m_receiveThread;
    UdpFrameReceiver *m_receiver = nullptr;
    QTimer m_watchdog;
    QByteArray m_decodeJpeg;
    MjpegStream::State m_state = MjpegStream::State::Idle;

    quint64 m_frameCounter = 0;
    bool m_open = false;
    int m_requestedProfile = -1;
};
