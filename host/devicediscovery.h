#pragma once

#include <QHostAddress>
#include <QObject>
#include <QTimer>
#include <QUrl>

class QUdpSocket;

// 在局域网内广播查找 ESP32-CAM（协议见 docs/通信协议.md 第 1 节）。
// 每 2 秒广播一次，直到 stop()。每次收到合法回复都会发出 deviceFound，由调用方去重。
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
        quint16 controlPort = 0;
    };

    explicit DeviceDiscovery(QObject *parent = nullptr);

    void start();
    void stop();
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
};

Q_DECLARE_METATYPE(DeviceDiscovery::Device)
