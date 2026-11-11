#include "udpstream.h"

#include <QNetworkDatagram>
#include <QNetworkInterface>
#include <QUdpSocket>
#include <QVariant>
#include <cstring>
#include <utility>

#include <opencv2/imgcodecs.hpp>

namespace {
constexpr int kHeartbeatIntervalMs = 500;
constexpr int kWatchdogTimeoutMs = 1500;
constexpr int kMaxChunkPayload = 1400;
constexpr int kMaxFrameSize = 65536;
constexpr int kSlotLifetimeMs = 250;
constexpr int kReadBatchLimit = 256;

bool newer(quint16 seq, quint16 previous)
{
    const quint16 delta = quint16(seq - previous);
    return delta > 0 && delta < 32768;
}
} // namespace

UdpStream::UdpStream(QObject *parent)
    : QObject(parent), m_socket(new QUdpSocket(this))
{
    qRegisterMetaType<cv::Mat>();
    m_clock.start();
    m_heartbeatTimer.setInterval(kHeartbeatIntervalMs);
    m_heartbeatTimer.setTimerType(Qt::PreciseTimer);
    connect(&m_heartbeatTimer, &QTimer::timeout, this, &UdpStream::sendHeartbeat);
    m_watchdog.setInterval(kWatchdogTimeoutMs);
    m_watchdog.setTimerType(Qt::PreciseTimer);
    connect(&m_watchdog, &QTimer::timeout, this, &UdpStream::onTimeout);
    connect(m_socket, &QUdpSocket::readyRead, this, &UdpStream::onReadyRead);
}

UdpStream::~UdpStream()
{
    close();
}

void UdpStream::resetFrames()
{
    for (FrameSlot &slot : m_slots)
        slot = FrameSlot{};
    m_hasDelivered = false;
}

void UdpStream::open(const QUrl &url)
{
    close();
    m_url = url;
    m_peerAddress = QHostAddress(url.host());
    const int port = url.port(5006);
    if (m_peerAddress.protocol() != QAbstractSocket::IPv4Protocol || port < 1 || port > 65535) {
        setState(MjpegStream::State::Reconnecting, tr("UDP 地址需要有效 IPv4 和端口"));
        return;
    }
    m_peerPort = quint16(port);

    // 选择最具体的本地子网，避免 VPN 的大网段覆盖物理网卡。
    QHostAddress localBindAddr = QHostAddress::AnyIPv4;
    int longestPrefix = -1;
    for (const QNetworkInterface &iface : QNetworkInterface::allInterfaces()) {
        if (!(iface.flags() & QNetworkInterface::IsUp)) continue;
        for (const QNetworkAddressEntry &entry : iface.addressEntries()) {
            const int prefix = entry.prefixLength();
            if (entry.ip().protocol() == QAbstractSocket::IPv4Protocol && prefix >= 0
                && prefix > longestPrefix && m_peerAddress.isInSubnet(entry.ip(), prefix)) {
                longestPrefix = prefix;
                localBindAddr = entry.ip();
            }
        }
    }
    if (!m_socket->bind(localBindAddr, 0)) {
        setState(MjpegStream::State::Reconnecting, tr("UDP 绑定失败：%1").arg(m_socket->errorString()));
        return;
    }
    m_socket->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption, QVariant(128 * 1024));
    m_frameCounter = 0;
    m_heartbeatTimer.start();
    m_watchdog.start();
    setState(MjpegStream::State::Connecting, m_url.toString());
    sendHeartbeat();
}

void UdpStream::close()
{
    m_heartbeatTimer.stop();
    m_watchdog.stop();
    m_socket->close();
    resetFrames();
    m_deviceId = 0;
    setState(MjpegStream::State::Idle);
}

void UdpStream::sendHeartbeat()
{
    if (m_socket->state() == QAbstractSocket::BoundState)
        m_socket->writeDatagram(QByteArrayLiteral("VREQ"), m_peerAddress, m_peerPort);
}

void UdpStream::onTimeout()
{
    // 相机重启会让 frame_seq 归零；重发心跳的同时清除旧序号基准。
    resetFrames();
    setState(MjpegStream::State::Reconnecting, tr("视频中断，正在恢复…"));
    sendHeartbeat();
}

void UdpStream::onReadyRead()
{
    QByteArray latest;
    quint16 latestSeq = 0;
    quint8 latestDevice = 0;
    qint64 receivedMs = 0;
    int count = 0;
    while (m_socket->hasPendingDatagrams() && count++ < kReadBatchLimit) {
        const QNetworkDatagram datagram = m_socket->receiveDatagram(sizeof(VidPacketHeader) + kMaxChunkPayload + 1);
        if (datagram.senderAddress() != m_peerAddress || datagram.senderPort() != m_peerPort)
            continue;
        const QByteArray &data = datagram.data();
        if (data.size() < int(sizeof(VidPacketHeader))) continue;
        VidPacketHeader hdr;
        std::memcpy(&hdr, data.constData(), sizeof(hdr));
        if (hdr.magic != 0x31444956 || hdr.device_id < 1 || hdr.device_id > 3
            || (m_deviceId != 0 && hdr.device_id != m_deviceId)
            || hdr.frame_size == 0 || hdr.frame_size > kMaxFrameSize)
            continue;
        const int expectedCount = (int(hdr.frame_size) + kMaxChunkPayload - 1) / kMaxChunkPayload;
        if (hdr.chunk_count != expectedCount || hdr.chunk_index >= hdr.chunk_count) continue;
        const int offset = hdr.chunk_index * kMaxChunkPayload;
        const int expectedPayload = qMin(kMaxChunkPayload, int(hdr.frame_size) - offset);
        if (hdr.payload_size != expectedPayload || data.size() != int(sizeof(hdr)) + expectedPayload)
            continue;
        if ((m_hasDelivered && !newer(hdr.frame_seq, m_lastDeliveredSeq))
            || (!latest.isEmpty() && !newer(hdr.frame_seq, latestSeq)))
            continue;

        const qint64 now = m_clock.elapsed();
        FrameSlot &slot = m_slots[hdr.frame_seq % kSlotCount];
        if (slot.active && now - slot.startedMs >= kSlotLifetimeMs) slot.active = false;
        if (slot.active && slot.seq != hdr.frame_seq && !newer(hdr.frame_seq, slot.seq)) continue;
        if (!slot.active || slot.seq != hdr.frame_seq) {
            slot.active = true;
            slot.seq = hdr.frame_seq;
            slot.chunkCount = hdr.chunk_count;
            slot.frameSize = hdr.frame_size;
            slot.buffer.resize(int(hdr.frame_size));
            slot.chunksReceived.fill(false, hdr.chunk_count);
            slot.receivedChunks = 0;
            slot.startedMs = now;
        } else if (slot.chunkCount != hdr.chunk_count || slot.frameSize != hdr.frame_size) {
            continue;
        }
        if (!slot.chunksReceived[hdr.chunk_index]) {
            std::memcpy(slot.buffer.data() + offset, data.constData() + sizeof(hdr), expectedPayload);
            slot.chunksReceived[hdr.chunk_index] = true;
            ++slot.receivedChunks;
        }
        if (slot.receivedChunks == slot.chunkCount) {
            latest = std::move(slot.buffer);
            latestSeq = hdr.frame_seq;
            latestDevice = hdr.device_id;
            receivedMs = now;
            slot.active = false;
        }
    }

    // 突发到达时只解码最新完整图像，避免逐张解码旧帧形成延迟队列。
    if (!latest.isEmpty()) {
        const cv::Mat raw(1, int(latest.size()), CV_8UC1, latest.data());
        cv::Mat bgr;
        try { bgr = cv::imdecode(raw, cv::IMREAD_COLOR); }
        catch (const cv::Exception &) { /* 损坏图像不推进序号、不续看门狗。 */ }
        if (!bgr.empty()) {
            m_hasDelivered = true;
            m_lastDeliveredSeq = latestSeq;
            m_deviceId = latestDevice;
            m_watchdog.start();
            if (m_state != MjpegStream::State::Streaming)
                setState(MjpegStream::State::Streaming, tr("%1×%2 (UDP)").arg(bgr.cols).arg(bgr.rows));
            emit frameDecoded(bgr, ++m_frameCounter, receivedMs);
        }
    }
    if (m_socket->hasPendingDatagrams())
        QMetaObject::invokeMethod(this, &UdpStream::onReadyRead, Qt::QueuedConnection);
}

void UdpStream::setState(MjpegStream::State state, const QString &detail)
{
    if (m_state == state && detail.isEmpty()) return;
    m_state = state;
    emit stateChanged(m_state, detail);
}
