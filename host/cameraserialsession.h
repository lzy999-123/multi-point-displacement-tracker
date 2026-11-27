#pragma once

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QFutureWatcher>
#include <QTimer>

// Optional, explicitly configured USB baseboard. Lifetime follows the app.
class CameraSerialSession : public QObject
{
    Q_OBJECT
public:
    explicit CameraSerialSession(const QString &portName, QObject *parent = nullptr);
signals:
    void dataReceived(const QByteArray &data);
private:
    void tryOpen();
    QString m_portName;
    QSerialPort m_port;
    QTimer m_retry;
    QFutureWatcher<QSerialPortInfo> m_scan;
};
