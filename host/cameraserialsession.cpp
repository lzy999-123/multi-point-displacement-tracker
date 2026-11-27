#include "cameraserialsession.h"

#include <QDebug>
#include <QSerialPortInfo>
#include <QtConcurrent/QtConcurrentRun>

CameraSerialSession::CameraSerialSession(const QString &portName, QObject *parent)
    : QObject(parent), m_portName(portName)
{
    if (m_portName.isEmpty()) return; // 独立供电时不占用任何串口。
    m_retry.setInterval(2000);
    connect(&m_retry, &QTimer::timeout, this, &CameraSerialSession::tryOpen);
    connect(&m_port, &QSerialPort::readyRead, this, [this] { emit dataReceived(m_port.readAll()); });
    connect(&m_port, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError error) {
        if (error == QSerialPort::ResourceError) m_port.close();
    });
    connect(&m_scan, &QFutureWatcher<QSerialPortInfo>::finished, this, [this] {
        if (m_port.isOpen()) return;
        const auto info = m_scan.result();
        if (info.isNull()) return;
        m_port.setPort(info);
        m_port.setBaudRate(115200);
        m_port.setFlowControl(QSerialPort::NoFlowControl);
        if (m_port.open(QIODevice::ReadWrite)) {
            m_port.setDataTerminalReady(false);
            m_port.setRequestToSend(false);
            qInfo() << "Camera USB session opened:" << info.portName();
        }
    });
    tryOpen();
    m_retry.start();
}

void CameraSerialSession::tryOpen()
{
    if (m_port.isOpen() || m_scan.isRunning()) return;
    const QString name = m_portName;
    m_scan.setFuture(QtConcurrent::run([name] {
        for (const auto &info : QSerialPortInfo::availablePorts()) {
            // Only open the explicitly configured CH340; exclude other UARTs.
            if (info.portName().compare(name, Qt::CaseInsensitive) == 0
                && info.vendorIdentifier() == 0x1a86 && info.productIdentifier() == 0x7523)
                return info;
        }
        return QSerialPortInfo();
    }));
}
