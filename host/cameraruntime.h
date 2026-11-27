#pragma once

#include <QObject>
#include <QFutureWatcher>
#include <QSet>
#include <QTimer>
#include <QUrl>
#include <QVector>

class CameraWifiSession;

// Shared by the normal app and real-window acceptance tests.
class CameraRuntime : public QObject
{
    Q_OBJECT
public:
    explicit CameraRuntime(const QString &settingsPath, bool configuredStreams = true,
                           QObject *parent = nullptr);
    int configuredStreamCount() const { return int(m_streams.size()); }
    void start();

signals:
    void streamReady(int index, const QUrl &url);
    void streamUnavailable(int index, const QString &reason);
    void serialDataReceived(int index, const QByteArray &data);

private:
    void refreshStreams(bool reconnect = false);
    void scanPorts();
    struct Stream {
        int index;
        QUrl url;
        QString serialPort;
        bool active = false;
        bool missingAnnounced = false;
    };
    QVector<Stream> m_streams;
    CameraWifiSession *m_wifi = nullptr;
    bool m_configuredStreams;
    bool m_networkReady = false;
    bool m_presenceKnown = false;
    QSet<QString> m_presentPorts;
    QFutureWatcher<QSet<QString>> m_portScan;
    QTimer m_presenceTimer;
};
