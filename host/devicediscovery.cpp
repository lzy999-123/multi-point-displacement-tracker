#include "devicediscovery.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkDatagram>
#include <QNetworkInterface>
#include <QRandomGenerator>
#include <QUdpSocket>

DeviceDiscovery::DeviceDiscovery(QObject *parent)
    : QObject(parent), m_socket(new QUdpSocket(this))
{
    m_timer.setInterval(2000);
    connect(&m_timer, &QTimer::timeout, this, &DeviceDiscovery::broadcast);
    connect(m_socket, &QUdpSocket::readyRead, this, &DeviceDiscovery::readPending);
}

void DeviceDiscovery::start()
{
    if (m_socket->state() != QAbstractSocket::BoundState)
        m_socket->bind(QHostAddress::AnyIPv4, 0);
    m_nonce = QString::number(QRandomGenerator::global()->generate64(), 16);
    broadcast();
    m_timer.start();
}

void DeviceDiscovery::stop()
{
    m_timer.stop();
}

void DeviceDiscovery::broadcast()
{
    const QByteArray request = QJsonDocument(QJsonObject{
        {"type", "discover"}, {"version", 1}, {"nonce", m_nonce}}).toJson(QJsonDocument::Compact);

    // Windows 上 255.255.255.255 只从一个网卡发出，所以对每个网卡的广播地址也发一次
    m_socket->writeDatagram(request, QHostAddress::Broadcast, kPort);
    const auto interfaces = QNetworkInterface::allInterfaces();
    for (const QNetworkInterface &iface : interfaces) {
        const auto flags = iface.flags();
        if (!(flags & QNetworkInterface::IsUp) || !(flags & QNetworkInterface::IsRunning)
            || (flags & QNetworkInterface::IsLoopBack))
            continue;
        for (const QNetworkAddressEntry &entry : iface.addressEntries()) {
            if (entry.ip().protocol() == QAbstractSocket::IPv4Protocol && !entry.broadcast().isNull())
                m_socket->writeDatagram(request, entry.broadcast(), kPort);
        }
    }
}

void DeviceDiscovery::readPending()
{
    while (m_socket->hasPendingDatagrams()) {
        const QNetworkDatagram datagram = m_socket->receiveDatagram(2048);
        Device device;
        if (parseAnnounce(datagram.data(), m_nonce, datagram.senderAddress(), &device))
            emit deviceFound(device);
    }
}

bool DeviceDiscovery::parseAnnounce(const QByteArray &datagram, const QString &nonce,
                                    const QHostAddress &sender, Device *device)
{
    QJsonParseError error;
    const QJsonDocument doc = QJsonDocument::fromJson(datagram, &error);
    if (error.error != QJsonParseError::NoError || !doc.isObject())
        return false;
    const QJsonObject obj = doc.object();
    const int ver = obj.value("version").toInt();
    if (obj.value("type").toString() != "announce" || (ver != 1 && ver != 2)
        || obj.value("nonce").toString() != nonce)
        return false;

    const int id = obj.value("id").toInt(0);
    if (id < 1 || id > kMaxDeviceId)
        return false;

    QUrl url(obj.value("stream").toString());
    if (!url.isValid() || url.scheme() != "http" || url.port() <= 0)
        return false;

    // 去掉 IPv4 映射前缀（::ffff:）
    bool isV4 = false;
    const quint32 v4 = sender.toIPv4Address(&isV4);
    if (!isV4)
        return false;
    const QHostAddress address(v4);
    url.setHost(address.toString());

    const int controlPort = obj.value("control_port").toInt(0);
    const int udpPort = obj.value("udp_stream").toInt(0);
    device->id = id;
    device->address = address;
    device->streamUrl = url;
    device->udpStreamPort = (udpPort > 0 && udpPort < 65536) ? quint16(udpPort) : 0;
    device->controlPort = controlPort > 0 && controlPort < 65536 ? quint16(controlPort) : 0;
    return true;
}
