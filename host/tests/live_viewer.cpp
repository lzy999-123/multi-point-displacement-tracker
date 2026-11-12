// Hardware acceptance: uses the real MainWindow, Channel, decoder and paint path.
#include <QApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTimer>
#include <QSettings>
#include <algorithm>
#include "mainwindow.h"
#include "videowidget.h"
#include "cameraserialsession.h"
#include "camerawifisession.h"
#include "cameraconfigpath.h"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    if (argc < 4) return 2; // URL, duration seconds, output JSON
    const QStringList arguments = app.arguments();
    const bool automatic = arguments[1] == QStringLiteral("auto");
    QSettings settings(cameraSettingsPath(), QSettings::IniFormat);
    const QUrl url(automatic ? settings.value("camera/url", "udp://192.168.4.1:5006").toString() : arguments[1]);
    const int seconds = arguments[2].toInt();
    const QString output = arguments[3];
    const int minimumFps = arguments.size() > 4 ? arguments[4].toInt() : 20;
    const int maximumGapMs = arguments.size() > 5 ? arguments[5].toInt() : 150;
    // 自动启动允许相机上电及 WLAN 切换；启动时间仍通过 first_ms 单独报告。
    const int warmupMs = automatic ? 10000 : 2000;
    if (!url.isValid() || seconds < warmupMs / 1000 + 3 || minimumFps < 1 || maximumGapMs < 1) return 2;
    CameraSerialSession serial(automatic ? settings.value("camera/serialPort").toString() : QString());
    CameraWifiSession wifi(automatic ? settings.value("camera/wifiProfile").toString() : QString());
    QFile serialLog(output + ".serial.log");
    if (automatic && serialLog.open(QIODevice::WriteOnly))
        QObject::connect(&serial, &CameraSerialSession::dataReceived, &app, [&](const QByteArray &bytes) {
            serialLog.write(bytes); serialLog.flush();
        });
    MainWindow window;
    // A single-camera test must have only one receiver. Discovery could otherwise
    // open the same camera in its device-ID channel and steal its UDP destination.
    for (int index = 1; index < MainWindow::kChannelCount; ++index)
        window.setChannelUnavailable(index, QStringLiteral("本次仅测试通道 1"));
    // 验收窗口保持可见，避免被其他窗口遮挡后 Windows 暂停绘制造成假卡顿。
    window.setWindowFlag(Qt::WindowStaysOnTopHint);
    const auto channels = window.findChildren<Channel *>();
    const auto videos = window.findChildren<VideoWidget *>();
    Channel *channel = nullptr;
    for (Channel *candidate : channels) if (candidate->id() == 1) channel = candidate;
    if (!channel || videos.isEmpty()) return 2;
    QVector<qint64> decoded, painted;
    int frameWidth = 0, frameHeight = 0;
    bool wrongResolution = false;
    QJsonArray states;
    QElapsedTimer clock;
    clock.start();
    QObject::connect(channel, &Channel::resultReady, &window, [&](const ChannelResult &result) {
        decoded.append(clock.elapsed());
        frameWidth = result.image.width();
        frameHeight = result.image.height();
        wrongResolution |= frameWidth != 320 || frameHeight != 240;
    });
    QObject::connect(videos.first(), &VideoWidget::framePresented, &window, [&](quint64) {
        painted.append(clock.elapsed());
    });
    QObject::connect(channel, &Channel::stateChanged, &window, [&](MjpegStream::State s, const QString &detail) {
        states.append(QJsonObject{{"ms", clock.elapsed()}, {"state", int(s)}, {"detail", detail}});
    });
    window.show();
    if (automatic) {
        QObject::connect(&wifi, &CameraWifiSession::ready, &window, [&window, url] { window.openChannel(0, url); });
        wifi.start();
    } else {
        QTimer::singleShot(0, &window, [&window, url] { window.openChannel(0, url); });
    }
    QTimer::singleShot(seconds * 1000, Qt::PreciseTimer, &window, [&] {
        const auto summarize = [seconds, warmupMs](const QVector<qint64> &times) {
            QVector<int> bins(seconds, 0);
            qint64 previous = warmupMs, maxGap = 0;
            for (qint64 time : times) {
                if (time / 1000 < seconds) ++bins[int(time / 1000)];
                if (time >= warmupMs && time < seconds * 1000LL) { maxGap = qMax(maxGap, time - previous); previous = time; }
            }
            maxGap = qMax(maxGap, seconds * 1000LL - previous);
            QJsonArray array;
            for (int n : bins) array.append(n);
            const int minimum = *std::min_element(bins.begin() + warmupMs / 1000, bins.end());
            return QJsonObject{{"frames", times.size()}, {"per_second", array},
                               {"min_fps_after_warmup", minimum}, {"max_gap_ms_after_warmup", maxGap},
                               {"first_ms", times.isEmpty() ? -1 : times.first()}};
        };
        const QJsonObject decode = summarize(decoded), paint = summarize(painted);
        QJsonArray decodeTimes, paintTimes;
        for (qint64 time : decoded) decodeTimes.append(time);
        for (qint64 time : painted) paintTimes.append(time);
        bool reconnect = false;
        for (const auto &state : states)
            if (state.toObject()["state"].toInt() == int(MjpegStream::State::Reconnecting)) reconnect = true;
        const bool passed = paint["min_fps_after_warmup"].toInt() >= minimumFps &&
                            paint["max_gap_ms_after_warmup"].toInteger() <= maximumGapMs &&
                            !reconnect && !wrongResolution && frameWidth == 320 && frameHeight == 240;
        const QJsonObject report{{"url", url.toString()}, {"duration_s", seconds},
                                 {"decoded", decode}, {"painted", paint},
                                 {"decoded_times_ms", decodeTimes}, {"painted_times_ms", paintTimes},
                                 {"width", frameWidth}, {"height", frameHeight},
                                 {"wrong_resolution_observed", wrongResolution},
                                 {"criteria", QJsonObject{{"min_fps_each_second", minimumFps},
                                                         {"max_gap_ms", maximumGapMs},
                                                         {"warmup_s", warmupMs / 1000},
                                                         {"required_width", 320}, {"required_height", 240}}},
                                 {"states", states}, {"passed", passed}};
        QFile file(output);
        if (!file.open(QIODevice::WriteOnly)) { app.exit(3); return; }
        file.write(QJsonDocument(report).toJson(QJsonDocument::Indented));
        file.close();
        window.grab().save(output + ".png");
        channel->close();
        app.exit(passed ? 0 : 1);
    });
    return app.exec();
}
