// Real two-camera display evidence using the production startup and paint path.
#include <QApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSettings>
#include <QTimer>
#include <algorithm>
#include <array>
#include "cameraruntime.h"
#include "cameraconfigpath.h"
#include "mainwindow.h"
#include "videowidget.h"

struct Samples {
    QVector<qint64> decoded, painted;
    QJsonArray states;
    int width = 0, height = 0;
    bool wrongResolution = false, reconnect = false;
};

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    if (argc != 3 && argc != 4) return 2; // duration, output JSON, optional one-connected
    const bool oneConnected = argc == 4 && app.arguments()[3] == "one-connected";
    if (argc == 4 && !oneConnected) return 2;
    bool valid = false;
    const int seconds = app.arguments()[1].toInt(&valid);
    constexpr int warmupMs = 10000;
    if (!valid || seconds < 15 || seconds > 3600) return 2;
    const QString output = app.arguments()[2];
    const QString settingsPath = cameraSettingsPath();
    QSettings settings(settingsPath, QSettings::IniFormat);
    const std::array<QUrl, 2> urls{QUrl(settings.value("camera/url").toString()),
                                  QUrl(settings.value("camera2/url").toString())};
    if (urls[0].host().isEmpty() || urls[1].host().isEmpty() || urls[0].host() == urls[1].host()) return 2;
    CameraRuntime runtime(settingsPath);
    if (runtime.configuredStreamCount() != 2) return 2;
    std::array<QFile, 2> serialLogs;
    for (int i = 0; i < 2; ++i) {
        serialLogs[i].setFileName(output + QStringLiteral(".channel%1.serial.log").arg(i + 1));
        if (!serialLogs[i].open(QIODevice::WriteOnly)) return 3;
    }
    QObject::connect(&runtime, &CameraRuntime::serialDataReceived, &app,
                     [&](int index, const QByteArray &data) {
        if (index < 2) { serialLogs[index].write(data); serialLogs[index].flush(); }
    });
    MainWindow window;
    window.setWindowFlag(Qt::WindowStaysOnTopHint);
    std::array<Channel *, 2> channels{};
    std::array<VideoWidget *, 2> videos{};
    std::array<Samples, 2> samples;
    QElapsedTimer clock;
    clock.start();
    for (int i = 0; i < 2; ++i) {
        for (Channel *channel : window.findChildren<Channel *>())
            if (channel->id() == i + 1) channels[i] = channel;
        videos[i] = window.findChild<VideoWidget *>(QStringLiteral("video-%1").arg(i + 1));
        if (!channels[i] || !videos[i]) return 2;
        QObject::connect(channels[i], &Channel::resultReady, &window, [&, i](const ChannelResult &r) {
            auto &sample = samples[i];
            sample.decoded.append(clock.elapsed());
            sample.width = r.image.width(); sample.height = r.image.height();
            sample.wrongResolution |= sample.width != 320 || sample.height != 240;
        });
        QObject::connect(videos[i], &VideoWidget::framePresented, &window, [&, i](quint64) {
            samples[i].painted.append(clock.elapsed());
        });
        QObject::connect(channels[i], &Channel::stateChanged, &window,
                         [&, i](MjpegStream::State state, const QString &detail) {
            samples[i].states.append(QJsonObject{{"ms", clock.elapsed()}, {"state", int(state)}, {"detail", detail}});
            samples[i].reconnect |= state == MjpegStream::State::Reconnecting;
        });
    }
    QObject::connect(&runtime, &CameraRuntime::streamReady, &window, &MainWindow::openChannel);
    QObject::connect(&runtime, &CameraRuntime::streamUnavailable, &window, &MainWindow::setChannelUnavailable);
    window.show();
    runtime.start();
    QTimer::singleShot(seconds * 1000, Qt::PreciseTimer, &window, [&] {
        const auto summarize = [seconds](const QVector<qint64> &times) {
            QVector<int> bins(seconds, 0);
            qint64 previous = warmupMs, gap = 0;
            for (qint64 time : times) {
                if (time < seconds * 1000LL) ++bins[int(time / 1000)];
                if (time >= warmupMs && time < seconds * 1000LL) {
                    gap = qMax(gap, time - previous); previous = time;
                }
            }
            gap = qMax(gap, seconds * 1000LL - previous);
            QJsonArray counts, raw;
            for (int count : bins) counts.append(count);
            for (qint64 time : times) raw.append(time);
            return QJsonObject{{"frames", times.size()}, {"per_second", counts}, {"times_ms", raw},
                {"min_fps_after_warmup", *std::min_element(bins.begin() + warmupMs / 1000, bins.end())},
                {"max_gap_ms_after_warmup", gap}, {"first_ms", times.isEmpty() ? -1 : times.first()}};
        };
        QJsonArray reports;
        bool simultaneous = true, strict = true;
        for (int i = 0; i < 2; ++i) {
            const auto &sample = samples[i];
            const QJsonObject decoded = summarize(sample.decoded), painted = summarize(sample.painted);
            const bool resolution = !sample.wrongResolution && sample.width == 320 && sample.height == 240;
            const bool display = resolution && painted["min_fps_after_warmup"].toInt() >= 1 && !sample.reconnect;
            const bool performance = display && painted["min_fps_after_warmup"].toInt() >= 20 &&
                                     painted["max_gap_ms_after_warmup"].toInteger() <= 150;
            simultaneous &= display; strict &= performance;
            reports.append(QJsonObject{{"id", i + 1}, {"url", urls[i].toString()},
                {"rotate180", videos[i]->rotate180()}, {"width", sample.width}, {"height", sample.height},
                {"wrong_resolution_observed", sample.wrongResolution}, {"reconnect_observed", sample.reconnect},
                {"decoded", decoded}, {"painted", painted}, {"states", sample.states},
                {"display_passed", display}, {"strict_20fps_passed", performance}});
        }
        const bool missingClosed = !channels[1]->isOpen() && samples[1].decoded.isEmpty();
        const bool connectedPassed = oneConnected
            ? reports[0].toObject()["display_passed"].toBool() && missingClosed : simultaneous;
        const QJsonObject report{{"duration_s", seconds}, {"warmup_s", warmupMs / 1000},
            {"settings_path", settingsPath}, {"required_active_camera_count", oneConnected ? 1 : 2},
            {"expected_missing_channel_closed", oneConnected && missingClosed},
            {"connected_camera_display_passed", connectedPassed},
            {"criteria", QJsonObject{{"functional_min_new_paints_each_second_per_camera", 1},
                {"strict_min_new_paints_each_second_per_camera", 20}, {"strict_max_gap_ms", 150},
                {"required_camera_count", oneConnected ? 1 : 2}, {"required_width", 320}, {"required_height", 240}}},
            {"channels", reports}, {"simultaneous_display_passed", simultaneous}, {"strict_20fps_passed", strict}};
        QFile file(output);
        if (!file.open(QIODevice::WriteOnly)) { app.exit(3); return; }
        file.write(QJsonDocument(report).toJson(QJsonDocument::Indented)); file.close();
        window.grab().save(output + ".png");
        for (auto *channel : channels) channel->close();
        app.exit(connectedPassed ? 0 : 1);
    });
    return app.exec();
}
