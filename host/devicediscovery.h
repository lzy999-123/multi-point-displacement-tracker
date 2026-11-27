#pragma once

#include <QHostAddress>
#include <QFutureWatcher>
#include <QHash>
#include <QObject>
#include <QTimer>
#include <QUrl>

class QUdpSocket;

// 广播查找 ESP32-CAM，并对已配置地址补发单播，兼容限制客户端广播的热点。
// 默认每 2 秒查询，直到 stop()。合法回复发出 deviceFound，由调用方去重。
class DeviceDiscovery : public QObject
{
    Q_OBJECT
public:
    static constexpr quint16 kPort = 39093;
    static constexpr int kMaxDeviceId = 3;

    struct Device {
        int id = 0;
        QHostAddress address;
        QUrl streamUrl;
        quint16 udpStreamPort = 0;
        quint16 controlPort = 0;

        QUrl bestStreamUrl() const {
            if (udpStreamPort > 0) {
                QUrl url;
                url.setScheme(QStringLiteral("udp"));
                url.setHost(address.toString());
                url.setPort(udpStreamPort);
                return url;
            }
            return streamUrl;
        }
    };

    explicit DeviceDiscovery(QObject *parent = nullptr);

    void start();
    void stop();
    void setTarget(int deviceId, const QHostAddress &address);
    void setInterval(int ms) { m_timer.setInterval(ms); }
    bool isRunning() const { return m_timer.isActive(); }

    // 解析一条回复；nonce 不符、字段缺失或 id 越界时返回 false。
    // 视频地址的主机部分强制使用 sender，避免设备回报的 IP 与实际来源不一致。
    static bool parseAnnounce(const QByteArray &datagram, const QString &nonce,
                              const QHostAddress &sender, Device *device);

signals:
    void deviceFound(const DeviceDiscovery::Device &device);

private:
    void broadcast();
    void readPending();

    QUdpSocket *m_socket = nullptr;
    QTimer m_timer;
    QString m_nonce;
    QHash<int, QHostAddress> m_targets;
    QList<QHostAddress> m_broadcasts;
    QFutureWatcher<QList<QHostAddress>> m_interfaceScan;
};

Q_DECLARE_METATYPE(DeviceDiscovery::Device)
