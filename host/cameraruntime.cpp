#include "cameraruntime.h"

#include <QDebug>
#include <QSet>
#include <QSettings>
#include <QSerialPortInfo>
#include "cameraserialsession.h"
#include "camerawifisession.h"

CameraRuntime::CameraRuntime(const QString &settingsPath, bool configuredStreams, QObject *parent)
    : QObject(parent), m_configuredStreams(configuredStreams)
{
    QSettings settings(settingsPath, QSettings::IniFormat);
    QSet<QString> ports;
    for (int index = 0; index < 3; ++index) {
        const QString group = index == 0 ? QStringLiteral("camera")
                                        : QStringLiteral("camera%1").arg(index + 1);
        const QString port = settings.value(group + "/serialPort").toString().trimmed();
        if (!port.isEmpty() && !ports.contains(port.toUpper())) {
            ports.insert(port.toUpper());
            auto *serial = new CameraSerialSession(port, this);
            connect(serial, &CameraSerialSession::dataReceived, this,
                    [this, index](const QByteArray &data) { emit serialDataReceived(index, data); });
        }
        const QString text = settings.value(group + "/url").toString().trimmed();
        if (text.isEmpty()) continue;
        const QUrl url(text);
        if (!url.isValid() || url.host().isEmpty() ||
            (url.scheme() != QStringLiteral("http") && url.scheme() != QStringLiteral("udp"))) {
            qWarning() << "Invalid configured camera URL for channel" << index + 1;
            continue;
        }
        m_streams.append({index, url, port});
    }
    m_wifi = new CameraWifiSession(configuredStreams
                                     ? settings.value("camera/wifiProfile").toString() : QString(), this);
    connect(m_wifi, &CameraWifiSession::ready, this, [this] {
        m_networkReady = true;
        refreshStreams(true);
    });
    m_presenceTimer.setInterval(1000);
    connect(&m_presenceTimer, &QTimer::timeout, this, [this] { refreshStreams(); });
}

void CameraRuntime::start()
{
    refreshStreams();
    m_presenceTimer.start();
    m_wifi->start();
}

void CameraRuntime::refreshStreams(bool reconnect)
{
    if (!m_configuredStreams) return;
    QSet<QString> present;
    for (const auto &port : QSerialPortInfo::availablePorts())
        if (port.vendorIdentifier() == 0x1a86 && port.productIdentifier() == 0x7523)
            present.insert(port.portName().toUpper());
    for (auto &stream : m_streams) {
        if (!stream.serialPort.isEmpty() && !present.contains(stream.serialPort.toUpper())) {
            if (stream.active || !stream.missingAnnounced)
                emit streamUnavailable(stream.index, tr("设备已断开（%1）").arg(stream.serialPort));
            stream.active = false;
            stream.missingAnnounced = true;
        } else {
            stream.missingAnnounced = false;
            if (m_networkReady && (!stream.active || reconnect)) {
                stream.active = true;
                emit streamReady(stream.index, stream.url);
            }
        }
    }
}
