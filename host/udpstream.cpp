#include "udpstream.h"

#include <QElapsedTimer>
#include <QHostAddress>
#include <QMutex>
#include <QMutexLocker>
#include <QNetworkDatagram>
#include <QNetworkInterface>
#include <QUdpSocket>
#include <QVariant>
#include <QUrlQuery>
#include <QDebug>
#include <QRandomGenerator>
#include <QVector>
#include <cstring>
#include <functional>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include "videoprofile.h"

namespace {
constexpr int kHeartbeatIntervalMs = 500;
constexpr int kWatchdogTimeoutMs = 1500;
constexpr int kMaxChunkPayload = 1400;
constexpr int kMaxFrameSize = 65536;
constexpr int kSlotLifetimeMs = 500;
constexpr int kReadBatchLimit = 256;

bool newer(quint16 seq, quint16 previous)
{
    const quint16 delta = quint16(seq - previous);
    return delta > 0 && delta < 32768;
}
} // namespace

// Only takeLatest() crosses threads directly; its mailbox is protected by a mutex.
// Socket, timers and assembly slots always belong to the dedicated receive thread.
class UdpFrameReceiver : public QObject
{
public:
    explicit UdpFrameReceiver(std::function<void()> notify, std::function<void(int)> profileApplied)
        : m_socket(new QUdpSocket(this)), m_heartbeat(new QTimer(this)), m_notify(std::move(notify)),
          m_profileApplied(std::move(profileApplied))
    {
        m_clock.start();
        for (auto &slot : m_slots) slot.buffer.reserve(kMaxFrameSize);
        m_pendingJpeg.reserve(kMaxFrameSize);
        m_heartbeat->setInterval(kHeartbeatIntervalMs);
        m_heartbeat->setTimerType(Qt::PreciseTimer);
        connect(m_heartbeat, &QTimer::timeout, this, &UdpFrameReceiver::sendHeartbeat);
        connect(m_socket, &QUdpSocket::readyRead, this, &UdpFrameReceiver::onReadyRead);
        m_repairTimer = new QTimer(this);
        m_repairTimer->setInterval(8);
        m_repairTimer->setTimerType(Qt::PreciseTimer);
        connect(m_repairTimer, &QTimer::timeout, this, &UdpFrameReceiver::requestMissingChunks);
    }

    QString open(const QUrl &url)
    {
        close();
        m_peerAddress = QHostAddress(url.host());
        const int port = url.port(5006);
        if (m_peerAddress.protocol() != QAbstractSocket::IPv4Protocol || port < 1 || port > 65535)
            return tr("UDP 地址需要有效 IPv4 和端口");
        m_peerPort = quint16(port);
        m_rxPackets = m_completedFrames = m_overwrittenFrames = m_staleComplete = m_expiredSlots = 0;
        m_repairRequests = m_completedAfterRepair = 0;
        m_maxAssemblyMs = 0;
        m_reportMs = m_clock.elapsed();
        m_requestedProfile = videoProfileFromUrl(url);
        m_profileConfirmed = false;
        m_hqTuneRequested = false;
        m_hqTuneActive = false;
        m_profileHeartbeat.clear();
        m_hqHeartbeat.clear();
        m_profileGeneration = quint16(QRandomGenerator::global()->generate());
        const QUrlQuery query(url);
        m_transportFlags = query.queryItemValue(QStringLiteral("transport")) == QStringLiteral("simple") ? 3 : 0;
        const bool wantHq = query.queryItemValue(QStringLiteral("hq")) == QStringLiteral("1");
        bool hasExpectedId = false;
        const int parsedId = query.queryItemValue(QStringLiteral("id")).toInt(&hasExpectedId);
        m_expectedDeviceId = (hasExpectedId && parsedId >= 1 && parsedId <= 3) ? quint8(parsedId) : 0;
        const auto setting = [&query](const QString &key, int fallback, int minimum, int maximum) {
            bool ok = false;
            const int value = query.queryItemValue(key).toInt(&ok);
            return ok ? qBound(minimum, value, maximum) : fallback;
        };
        m_heartbeatPacket = QByteArrayLiteral("VREQA2");
        m_heartbeatPacket.append(char(setting(QStringLiteral("fps"), 28, 5, 40)));
        m_heartbeatPacket.append(char(setting(QStringLiteral("qmin"), 22, 10, 40)));
        m_heartbeatPacket.append(char(setting(QStringLiteral("qmax"), 38, 22, 62)));
        m_heartbeatPacket.append(char(setting(QStringLiteral("tos"), 0, 0, 255)));
        const int targetBytes = setting(QStringLiteral("bytes"), 6000, 3000, 16000);
        m_heartbeatPacket.append(char(targetBytes & 0xff));
        m_heartbeatPacket.append(char(targetBytes >> 8));
        if (m_requestedProfile >= 0) {
            const auto preset = VideoProfile::preset(unsigned(m_requestedProfile));
            m_heartbeatPacket[5] = '3';
            m_heartbeatPacket[6] = char(preset.fps);
            m_heartbeatPacket[7] = char(preset.minimumQuality);
            m_heartbeatPacket[8] = char(preset.maximumQuality);
            m_heartbeatPacket[10] = char(preset.targetBytes & 0xff);
            m_heartbeatPacket[11] = char(preset.targetBytes >> 8);
            m_heartbeatPacket.append(char(m_requestedProfile));
            m_heartbeatPacket.append(char(m_profileGeneration & 0xff));
            m_heartbeatPacket.append(char(m_profileGeneration >> 8));
            m_heartbeatPacket.append(char(m_transportFlags));
            m_profileHeartbeat = m_heartbeatPacket;
            if (wantHq && (m_transportFlags & 1)) {
                m_hqTuneRequested = true;
                bool hasHqq = false;
                const int hqq = query.queryItemValue(QStringLiteral("hqq")).toInt(&hasHqq);
                uint8_t qmin = preset.minimumQuality;
                uint8_t qmax = preset.maximumQuality;
                uint16_t hqTarget = preset.targetBytes;
                if (hasHqq && hqq >= 10 && hqq <= 40) {
                    qmin = uint8_t(hqq);
                    qmax = uint8_t(hqq);
                    hqTarget = 16000;
                } else if (hasHqq && hqq < 0 && m_requestedProfile == VideoProfile::Vga12) {
                    qmin = 18;
                    qmax = 42;
                    hqTarget = 10500;
                }
                m_hqHeartbeat = QByteArrayLiteral("VREQ02");
                m_hqHeartbeat.append(char(VideoProfile::hqTuneFps(unsigned(m_requestedProfile))));
                m_hqHeartbeat.append(char(qmin));
                m_hqHeartbeat.append(char(qmax));
                m_hqHeartbeat.append(char(0));
                m_hqHeartbeat.append(char(hqTarget & 0xff));
                m_hqHeartbeat.append(char(hqTarget >> 8));
            }
        }

        // Prefer the most specific subnet over a VPN's broad route.
        QHostAddress bindAddress = QHostAddress::AnyIPv4;
        int longestPrefix = -1;
        for (const auto &iface : QNetworkInterface::allInterfaces()) {
            if (!(iface.flags() & QNetworkInterface::IsUp)) continue;
            for (const auto &entry : iface.addressEntries()) {
                const int prefix = entry.prefixLength();
                if (entry.ip().protocol() == QAbstractSocket::IPv4Protocol && prefix >= 0
                    && prefix > longestPrefix && m_peerAddress.isInSubnet(entry.ip(), prefix)) {
                    longestPrefix = prefix;
                    bindAddress = entry.ip();
                }
            }
        }
        if (!m_socket->bind(bindAddress, 0))
            return tr("UDP 绑定失败：%1").arg(m_socket->errorString());
        m_socket->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption, QVariant(1024 * 1024));

        m_heartbeat->start();
        m_repairTimer->start();
        sendHeartbeat();
        return {};
    }

    void close()
    {
        m_heartbeat->stop();
        m_repairTimer->stop();
        if (m_socket->state() == QAbstractSocket::BoundState)
            m_socket->writeDatagram(QByteArrayLiteral("VSTP"), m_peerAddress, m_peerPort);

        m_socket->close();
        resetFrames();
    }

    void recover()
    {
        resetFrames();
        if (m_hqTuneRequested && !m_profileHeartbeat.isEmpty()) {
            // Re-lock sensor resolution via VREQA3 first in case the camera rebooted.
            m_profileConfirmed = false;
            m_hqTuneActive = false;
            m_heartbeatPacket = m_profileHeartbeat;
        }
        sendHeartbeat();
    }

    bool takeLatest(QByteArray &jpeg, qint64 &receivedMs)
    {
        QMutexLocker locker(&m_mutex);
        if (!m_pending) return false;
        jpeg.swap(m_pendingJpeg); // Return the previous decode buffer for reuse.
        receivedMs = m_receivedMs;
        m_pending = false;
        return true;
    }

private:
    struct FrameSlot {
        quint16 seq = 0;
        quint8 chunkCount = 0;
        quint8 receivedChunks = 0;
        quint32 frameSize = 0;
        QByteArray buffer;
        QVector<bool> chunksReceived;
        bool active = false;
        bool complete = false;
        qint64 startedMs = 0;
        qint64 lastPacketMs = 0;
        qint64 lastRepairMs = 0;
        int repairRequests = 0;
    };

    void resetFrames()
    {
        for (auto &slot : m_slots) slot.active = false;
        m_hasCompleted = false;
        m_deviceId = 0;
        QMutexLocker locker(&m_mutex);
        m_pending = false;
    }

    void sendHeartbeat()
    {
        if (m_socket->state() == QAbstractSocket::BoundState)
            m_socket->writeDatagram(m_heartbeatPacket, m_peerAddress, m_peerPort);
        const qint64 now = m_clock.elapsed();
        if (now - m_reportMs >= 5000) {
            qInfo().noquote() << QStringLiteral("UDP peer=%1 packets=%2 complete=%3 overwritten=%4 stale=%5 expired=%6 nack=%7 complete_after_nack=%8 assembly_max_ms=%9")
                .arg(m_peerAddress.toString()).arg(m_rxPackets).arg(m_completedFrames)
                .arg(m_overwrittenFrames).arg(m_staleComplete).arg(m_expiredSlots)
                .arg(m_repairRequests).arg(m_completedAfterRepair).arg(m_maxAssemblyMs);
            m_reportMs = now;
        }
    }

    void requestMissingChunks()
    {
        if (m_transportFlags & 2) return;
        const qint64 now = m_clock.elapsed();
        for (auto &slot : m_slots) {
            // Repair only fresh, mostly received frames, at most twice. Missing
            // an old image never holds newer complete images behind it.
            if (!slot.active || slot.complete || slot.repairRequests >= 2
                || now - slot.startedMs < 32 || now - slot.startedMs > 250
                || now - slot.lastPacketMs < 16 || now - slot.lastRepairMs < 64
                || (m_hasCompleted && !newer(slot.seq, m_lastCompletedSeq)))
                continue;
            quint64 missing = 0;
            int missingCount = 0;
            for (int i = 0; i < slot.chunkCount; ++i) {
                if (!slot.chunksReceived[i]) {
                    missing |= quint64(1) << i;
                    ++missingCount;
                }
            }
            if (missingCount == 0 || missingCount > 3) continue;
            QByteArray request = QByteArrayLiteral("VNCK");
            request.append(char(slot.seq & 0xff));
            request.append(char(slot.seq >> 8));
            for (int i = 0; i < 8; ++i) request.append(char(missing >> (i * 8)));
            m_socket->writeDatagram(request, m_peerAddress, m_peerPort);
            ++m_repairRequests;
            slot.lastRepairMs = now;
            ++slot.repairRequests;
        }
    }

    void onReadyRead()
    {
        int count = 0;
        while (m_socket->hasPendingDatagrams() && count++ < kReadBatchLimit) {
            const auto datagram = m_socket->receiveDatagram(sizeof(VidPacketHeader) + kMaxChunkPayload + 1);
            if (datagram.senderAddress() != m_peerAddress || datagram.senderPort() != m_peerPort) continue;
            ++m_rxPackets;
            const QByteArray &data = datagram.data();
            if (data.size() == 16 && data.startsWith("VCFG")) {
                const auto word = [&data](int index) {
                    return quint16(quint8(data[index])) | (quint16(quint8(data[index + 1])) << 8);
                };
                const auto preset = VideoProfile::preset(unsigned(m_requestedProfile));
                if (m_requestedProfile >= 0 && quint8(data[4]) >= 1 && quint8(data[4]) <= 3
                    && (m_expectedDeviceId == 0 || quint8(data[4]) == m_expectedDeviceId)
                    && quint8(data[5]) == m_requestedProfile && word(6) == m_profileGeneration
                    && word(8) == preset.width && word(10) == preset.height
                    && quint8(data[12]) == preset.fps && data[13] == 0
                    && quint8(data[14]) == m_transportFlags && !m_profileConfirmed) {
                    m_profileConfirmed = true;
                    if (m_hqTuneRequested && !m_hqHeartbeat.isEmpty()) {
                        m_hqTuneActive = true;
                        m_heartbeatPacket = m_hqHeartbeat;
                        if (m_socket->state() == QAbstractSocket::BoundState)
                            m_socket->writeDatagram(m_heartbeatPacket, m_peerAddress, m_peerPort);
                    }
                    m_profileApplied(m_requestedProfile);
                }
                continue;
            }
            if (data.size() < int(sizeof(VidPacketHeader))) continue;
            VidPacketHeader hdr;
            std::memcpy(&hdr, data.constData(), sizeof(hdr));
            const bool profileTagOk = m_requestedProfile < 0
                || hdr.reserved == m_requestedProfile + 1
                || (m_hqTuneActive && hdr.reserved == 0);
            if (hdr.magic != 0x31444956 || hdr.device_id < 1 || hdr.device_id > 3
                || (m_expectedDeviceId != 0 && hdr.device_id != m_expectedDeviceId)
                || (m_deviceId != 0 && hdr.device_id != m_deviceId)
                || !profileTagOk
                || hdr.frame_size == 0 || hdr.frame_size > kMaxFrameSize) continue;
            const int expectedCount = (int(hdr.frame_size) + kMaxChunkPayload - 1) / kMaxChunkPayload;
            if (hdr.chunk_count != expectedCount || hdr.chunk_index >= hdr.chunk_count) continue;
            const int offset = hdr.chunk_index * kMaxChunkPayload;
            const int expectedPayload = qMin(kMaxChunkPayload, int(hdr.frame_size) - offset);
            if (hdr.payload_size != expectedPayload || data.size() != int(sizeof(hdr)) + expectedPayload) continue;

            const qint64 now = m_clock.elapsed();
            FrameSlot *slot = nullptr;
            FrameSlot *available = nullptr;
            FrameSlot *oldest = &m_slots[0];
            // Sequence numbers may jump by any amount: never index by seq % 8.
            for (auto &candidate : m_slots) {
                if (candidate.active && now - candidate.startedMs >= kSlotLifetimeMs) {
                    if (!candidate.complete) ++m_expiredSlots;
                    candidate.active = false;
                }
                if (candidate.active && candidate.seq == hdr.frame_seq) slot = &candidate;
                if (!candidate.active && !available) available = &candidate;
                if (candidate.startedMs < oldest->startedMs) oldest = &candidate;
            }
            if (!slot) {
                slot = available ? available : oldest;
                slot->active = true;
                slot->complete = false;
                slot->seq = hdr.frame_seq;
                slot->chunkCount = hdr.chunk_count;
                slot->frameSize = hdr.frame_size;
                slot->buffer.resize(int(hdr.frame_size));
                slot->chunksReceived.fill(false, hdr.chunk_count);
                slot->receivedChunks = 0;
                slot->startedMs = now;
                slot->lastRepairMs = 0;
                slot->repairRequests = 0;
            } else if (slot->chunkCount != hdr.chunk_count || slot->frameSize != hdr.frame_size) {
                continue;
            }
            if (slot->complete) continue;
            slot->lastPacketMs = now;
            if (!slot->chunksReceived[hdr.chunk_index]) {
                std::memcpy(slot->buffer.data() + offset, data.constData() + sizeof(hdr), expectedPayload);
                slot->chunksReceived[hdr.chunk_index] = true;
                ++slot->receivedChunks;
            }
            if (slot->receivedChunks != slot->chunkCount) continue;
            slot->complete = true;
            ++m_completedFrames;
            if (slot->repairRequests) ++m_completedAfterRepair;
            m_maxAssemblyMs = qMax(m_maxAssemblyMs, now - slot->startedMs);
            m_deviceId = hdr.device_id;

            // VACK confirms assembly, including frames skipped by latest-only decode.
            // It does not promise that the JPEG decoded or appeared on the screen.
            QByteArray ack = QByteArrayLiteral("VACK");
            ack.append(char(hdr.frame_seq & 0xff));
            ack.append(char(hdr.frame_seq >> 8));
            if (!(m_transportFlags & 1))
                m_socket->writeDatagram(ack, m_peerAddress, m_peerPort);
            if (m_hasCompleted && !newer(hdr.frame_seq, m_lastCompletedSeq)) {
                ++m_staleComplete;
                continue;
            }
            m_hasCompleted = true;
            m_lastCompletedSeq = hdr.frame_seq;

            bool notify = false;
            {
                QMutexLocker locker(&m_mutex);
                if (m_pending) ++m_overwrittenFrames;
                // Keep slot storage and both mailbox/decode buffers allocated.
                m_pendingJpeg.resize(slot->buffer.size());
                std::memcpy(m_pendingJpeg.data(), slot->buffer.constData(), size_t(slot->buffer.size()));
                m_receivedMs = now;
                notify = !m_pending;
                m_pending = true;
            }
            if (notify) m_notify();
        }
        // readyRead need not fire again for packets left by the bounded batch.
        if (m_socket->hasPendingDatagrams() && !m_readScheduled) {
            m_readScheduled = true;
            QMetaObject::invokeMethod(this, [this] {
                m_readScheduled = false;
                onReadyRead();
            }, Qt::QueuedConnection);
        }
    }

    QUdpSocket *m_socket;
    QTimer *m_heartbeat;
    QTimer *m_repairTimer;
    QByteArray m_heartbeatPacket;
    QByteArray m_profileHeartbeat;
    QByteArray m_hqHeartbeat;
    const std::function<void()> m_notify;
    const std::function<void(int)> m_profileApplied;
    int m_requestedProfile = -1;
    quint16 m_profileGeneration = 0;
    bool m_profileConfirmed = false;
    bool m_hqTuneRequested = false;
    bool m_hqTuneActive = false;
    quint8 m_transportFlags = 0;
    QElapsedTimer m_clock;
    QHostAddress m_peerAddress;
    quint16 m_peerPort = 5006;
    FrameSlot m_slots[8];
    bool m_hasCompleted = false;
    quint16 m_lastCompletedSeq = 0;
    quint8 m_deviceId = 0;
    quint8 m_expectedDeviceId = 0;
    bool m_readScheduled = false;
    quint64 m_rxPackets = 0, m_completedFrames = 0, m_overwrittenFrames = 0, m_staleComplete = 0;
    quint64 m_expiredSlots = 0, m_repairRequests = 0, m_completedAfterRepair = 0;
    qint64 m_reportMs = 0, m_maxAssemblyMs = 0;
    QMutex m_mutex;

    QByteArray m_pendingJpeg;
    qint64 m_receivedMs = 0;
    bool m_pending = false;
};

UdpStream::UdpStream(QObject *parent)
    : QObject(parent), m_receiver(new UdpFrameReceiver([this] {
          QMetaObject::invokeMethod(this, &UdpStream::decodePending, Qt::QueuedConnection);
      }, [this](int profile) {
          QMetaObject::invokeMethod(this, [this, profile] {
              if (m_open && profile == m_requestedProfile) emit videoProfileApplied(profile);
          }, Qt::QueuedConnection);
      }))
{
    qRegisterMetaType<cv::Mat>();
    m_decodeJpeg.reserve(kMaxFrameSize);
    m_receiver->moveToThread(&m_receiveThread);
    connect(&m_receiveThread, &QThread::finished, m_receiver, &QObject::deleteLater);
    m_receiveThread.setObjectName(QStringLiteral("udp-receive"));
    m_receiveThread.start();
    m_watchdog.setInterval(kWatchdogTimeoutMs);
    m_watchdog.setTimerType(Qt::PreciseTimer);
    connect(&m_watchdog, &QTimer::timeout, this, &UdpStream::onTimeout);
}

UdpStream::~UdpStream()
{
    close();
    m_receiveThread.quit();
    m_receiveThread.wait();
}

void UdpStream::open(const QUrl &url)
{
    close();
    QString error;
    QMetaObject::invokeMethod(m_receiver, [&] { error = m_receiver->open(url); }, Qt::BlockingQueuedConnection);
    if (!error.isEmpty()) {
        setState(MjpegStream::State::Reconnecting, error);
        return;
    }
    m_frameCounter = 0;
    m_requestedProfile = videoProfileFromUrl(url);
    m_open = true;
    m_watchdog.start();
    setState(MjpegStream::State::Connecting, url.toString());
}

void UdpStream::close()
{
    m_open = false;
    m_watchdog.stop();
    QMetaObject::invokeMethod(m_receiver, [this] { m_receiver->close(); }, Qt::BlockingQueuedConnection);
    setState(MjpegStream::State::Idle);
}

void UdpStream::onTimeout()
{
    // Only a decoded image feeds the watchdog; packets alone cannot hide a dead video stream.
    QMetaObject::invokeMethod(m_receiver, [this] { m_receiver->recover(); }, Qt::BlockingQueuedConnection);
    setState(MjpegStream::State::Reconnecting, tr("视频中断，正在恢复…"));
}

void UdpStream::decodePending()
{
    qint64 receivedMs = 0;
    if (!m_open || !m_receiver->takeLatest(m_decodeJpeg, receivedMs)) return;
    const cv::Mat raw(1, int(m_decodeJpeg.size()), CV_8UC1, m_decodeJpeg.data());
    cv::Mat bgr;
    try { bgr = cv::imdecode(raw, cv::IMREAD_COLOR); }
    catch (const cv::Exception &) { return; }
    if (bgr.empty()) return;
    if (m_requestedProfile >= 0) {
        const auto preset = VideoProfile::preset(unsigned(m_requestedProfile));
        if (bgr.cols != preset.width || bgr.rows != preset.height) return;
    }
    m_watchdog.start();
    if (m_state != MjpegStream::State::Streaming)
        setState(MjpegStream::State::Streaming,
                 m_requestedProfile >= 0
                     ? tr("%1×%2 · %3 (UDP)").arg(bgr.cols).arg(bgr.rows)
                           .arg(videoProfileCadence(m_requestedProfile))
                     : tr("%1×%2 (UDP)").arg(bgr.cols).arg(bgr.rows));
    emit frameDecoded(bgr, ++m_frameCounter, receivedMs);
}

void UdpStream::setState(MjpegStream::State state, const QString &detail)
{
    if (m_state == state && detail.isEmpty()) return;
    m_state = state;
    emit stateChanged(m_state, detail);
}
