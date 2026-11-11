#pragma once

#include <QElapsedTimer>
#include <QHostAddress>
#include <QObject>
#include <QTimer>
#include <QUrl>
#include <QVector>

#include <opencv2/core.hpp>

#include "mjpegstream.h"

class QUdpSocket;

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
// 有限槽位容忍分片乱序；每批报文只解码最新完整帧，断流后重新建立序号基准。
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

private:
    void sendHeartbeat();
    void onReadyRead();
    void onTimeout();
    void resetFrames();
    void setState(MjpegStream::State state, const QString &detail = {});

    QUdpSocket *m_socket = nullptr;
    QTimer m_heartbeatTimer;
    QTimer m_watchdog;
    QElapsedTimer m_clock;

    QUrl m_url;
    QHostAddress m_peerAddress;
    quint16 m_peerPort = 5006;
    MjpegStream::State m_state = MjpegStream::State::Idle;

    struct FrameSlot {
        quint16 seq = 0;
        quint8 chunkCount = 0;
        quint8 receivedChunks = 0;
        quint32 frameSize = 0;
        QByteArray buffer;
        QVector<bool> chunksReceived;
        bool active = false;
        qint64 startedMs = 0;
    };
    static constexpr int kSlotCount = 8;
    FrameSlot m_slots[kSlotCount];

    quint64 m_frameCounter = 0;
    quint16 m_lastDeliveredSeq = 0;
    bool m_hasDelivered = false;
    quint8 m_deviceId = 0;
};
