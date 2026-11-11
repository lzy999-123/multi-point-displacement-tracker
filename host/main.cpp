#include <QApplication>
#include <QCommandLineParser>
#include <QNetworkInterface>
#include <QLocalServer>
#include <QLocalSocket>
#include <QTimer>
#include <QDebug>
#include "cameraconfigpath.h"
#include "cameraruntime.h"

#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QCommandLineParser parser;
    parser.addHelpOption();
    QCommandLineOption streamUrl(QStringLiteral("stream-url"), QStringLiteral("连接通道1的视频地址（UDP/HTTP）"), QStringLiteral("url"));
    parser.addOption(streamUrl);
    QCommandLineOption quitAfter(QStringLiteral("quit-after"), QStringLiteral("诊断：指定秒数后正常退出"), QStringLiteral("seconds"));
    quitAfter.setFlags(QCommandLineOption::HiddenFromHelp);
    parser.addOption(quitAfter);
    parser.process(app);
    int diagnosticSeconds = 0;
    if (parser.isSet(quitAfter)) {
        bool valid = false;
        diagnosticSeconds = parser.value(quitAfter).toInt(&valid);
        if (!valid || diagnosticSeconds < 1 || diagnosticSeconds > 3600) return 2;
    }

    // 多个相机客户端会互相覆盖 ESP32 的推流目标；只保留一个正式窗口。
    const QString serverName = QStringLiteral("DisplacementTracker-camera-") + qEnvironmentVariable("USERNAME");
    QLocalSocket existing;
    existing.connectToServer(serverName);
    if (existing.waitForConnected(150)) {
        existing.write("activate");
        existing.waitForBytesWritten(150);
        return 0;
    }
    QLocalServer instance;
    instance.setSocketOptions(QLocalServer::UserAccessOption);
    if (!instance.listen(serverName)) return 1;
    const QString settingsPath = cameraSettingsPath();
    qInfo() << "Camera settings:" << settingsPath;
    CameraRuntime runtime(settingsPath, !parser.isSet(streamUrl));
    MainWindow window;
    QObject::connect(&runtime, &CameraRuntime::streamReady, &window, &MainWindow::openChannel);
    QObject::connect(&runtime, &CameraRuntime::streamUnavailable, &window, &MainWindow::setChannelUnavailable);
    runtime.start();
    QObject::connect(&instance, &QLocalServer::newConnection, &window, [&] {
        while (auto *client = instance.nextPendingConnection()) {
            client->close(); client->deleteLater();
        }
        window.showNormal(); window.raise(); window.activateWindow();
    });
    if (parser.isSet(streamUrl)) {
        const QUrl url(parser.value(streamUrl));
        const QString scheme = url.scheme().toLower();
        if ((scheme != QStringLiteral("http") && scheme != QStringLiteral("udp")) || url.host().isEmpty()) return 2;
        window.openChannel(0, url);
    } else if (runtime.configuredStreamCount() == 0) {
        // 直连热点下无需等待广播发现，也不误选路由器上联地址。
        for (const auto &address : QNetworkInterface::allAddresses()) {
            if (address.protocol() == QAbstractSocket::IPv4Protocol &&
                address.isInSubnet(QHostAddress(QStringLiteral("192.168.4.0")), 24)) {
                window.openChannel(0, QUrl(QStringLiteral("udp://192.168.4.1:5006")));
                break;
            }
        }
    }
    window.show();
    if (diagnosticSeconds > 0)
        QTimer::singleShot(diagnosticSeconds * 1000, &app, &QCoreApplication::quit);
    return app.exec();
}
