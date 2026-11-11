#include "cameraserialsession.h"

#include <QDebug>
#include <QSerialPortInfo>

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
    tryOpen();
    m_retry.start();
}

void CameraSerialSession::tryOpen()
{
    if (m_port.isOpen()) return;
    for (const auto &info : QSerialPortInfo::availablePorts()) {
        // 只操作本机配置的 CH340；不碰其他 USB/UART 或蓝牙串口。
        if (info.portName().compare(m_portName, Qt::CaseInsensitive) != 0
            || info.vendorIdentifier() != 0x1a86 || info.productIdentifier() != 0x7523)
            continue;
        m_port.setPort(info);
        m_port.setBaudRate(115200);
        m_port.setFlowControl(QSerialPort::NoFlowControl);
        if (m_port.open(QIODevice::ReadWrite)) {
            m_port.setDataTerminalReady(false);
            m_port.setRequestToSend(false);
            qInfo() << "Camera USB session opened:" << info.portName();
        }
        break;
    }
}
