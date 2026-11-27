// Real camera display evidence using the production startup and paint path.
#include <QApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSettings>
#include <QSet>
#include <QTimer>
#include <QWindow>
#include <algorithm>
#include <array>
#include <cstdio>
#include "cameraruntime.h"
#include "cameraconfigpath.h"
#include "mainwindow.h"
#include "videowidget.h"
#include "videoprofile.h"

struct Samples {
    QVector<qint64> decoded, painted;
    QJsonArray states;
    QJsonArray transports;
    QJsonArray confirmations;
    int width = 0, height = 0;
    bool wrongResolution = false, reconnect = false;
};

int main(int argc, char **argv)
{
    qInstallMessageHandler([](QtMsgType, const QMessageLogContext &, const QString &message) {
        const QByteArray text = message.toUtf8();
        std::fprintf(stderr, "%s\n", text.constData());
        std::fflush(stderr);
    });
    QApplication app(argc, argv);
    if (argc != 3 && argc != 4 && argc != 6) return 2; // duration, JSON, count, optional width/height
    const bool oneConnected = argc == 4 && app.arguments()[3] == "one-connected";
#ifdef BASELINE_DIAGNOSTIC
    const int cameraCount = argc >= 4 && !oneConnected ? app.arguments()[3].toInt() : 3;
#else
    const int cameraCount = argc >= 4 && !oneConnected ? app.arguments()[3].toInt() : 2;
#endif
    const int expectedWidth = argc == 6 ? app.arguments()[4].toInt() : 320;
    const int expectedHeight = argc == 6 ? app.arguments()[5].toInt() : 240;
    if (expectedWidth < 1 || expectedHeight < 1) return 2;
    if (cameraCount < 2 || cameraCount > MainWindow::kChannelCount) return 2;
    bool valid = false;
    const int seconds = app.arguments()[1].toInt(&valid);
    constexpr int warmupMs = 10000;
    if (!valid || seconds < 15 || seconds > 3600) return 2;
    const QString output = app.arguments()[2];
    const QString settingsPath = cameraSettingsPath();
    QSettings settings(settingsPath, QSettings::IniFormat);
    std::array<QUrl, MainWindow::kChannelCount> urls;
    QSet<QString> hosts;
    for (int i = 0; i < cameraCount; ++i) {
        const QString group = i == 0 ? QStringLiteral("camera") : QStringLiteral("camera%1").arg(i + 1);
        urls[i] = QUrl(settings.value(group + "/url").toString());
        if (urls[i].host().isEmpty() || hosts.contains(urls[i].host())) return 2;
        hosts.insert(urls[i].host());
    }
    CameraRuntime runtime(settingsPath);
    if (runtime.configuredStreamCount() != cameraCount) return 2;
    std::array<QFile, MainWindow::kChannelCount> serialLogs;
    for (int i = 0; i < cameraCount; ++i) {
        serialLogs[i].setFileName(output + QStringLiteral(".channel%1.serial.log").arg(i + 1));
        if (!serialLogs[i].open(QIODevice::WriteOnly)) return 3;
    }
    QObject::connect(&runtime, &CameraRuntime::serialDataReceived, &app,
                     [&](int index, const QByteArray &data) {
        if (index < cameraCount) { serialLogs[index].write(data); serialLogs[index].flush(); }
    });
    MainWindow window;
#ifdef BASELINE_DIAGNOSTIC
    window.setVideoProfile(VideoProfile::Baseline10);
#else
    window.setVideoProfile(expectedWidth == 320 ? VideoProfile::Qvga25 : VideoProfile::Vga12);
#endif
    window.setWindowFlag(Qt::WindowStaysOnTopHint);
    std::array<Channel *, MainWindow::kChannelCount> channels{};
    std::array<VideoWidget *, MainWindow::kChannelCount> videos{};
    std::array<Samples, MainWindow::kChannelCount> samples;
    QElapsedTimer clock;
    clock.start();
    QJsonArray visibility;
    bool visibilityInterrupted = false;
    QTimer visibilityTimer;
    visibilityTimer.setInterval(250);
    QObject::connect(&visibilityTimer, &QTimer::timeout, &window, [&] {
        const bool exposed = window.isVisible() && !window.isMinimized()
                             && window.windowHandle() && window.windowHandle()->isExposed();
        visibility.append(QJsonObject{{"ms", clock.elapsed()}, {"exposed", exposed},
                                       {"minimized", window.isMinimized()}});
        if (clock.elapsed() >= warmupMs && !exposed) visibilityInterrupted = true;
    });
    for (int i = cameraCount; i < MainWindow::kChannelCount; ++i)
        window.setChannelUnavailable(i, QStringLiteral("本次未测试此通道"));
    for (int i = 0; i < cameraCount; ++i) {
        for (Channel *channel : window.findChildren<Channel *>())
            if (channel->id() == i + 1) channels[i] = channel;
        videos[i] = window.findChild<VideoWidget *>(QStringLiteral("video-%1").arg(i + 1));
        if (!channels[i] || !videos[i]) return 2;
        QObject::connect(channels[i], &Channel::resultReady, &window, [&, i](const ChannelResult &r) {
            if (r.revision != channels[i]->revision()) return;
            auto &sample = samples[i];
            sample.decoded.append(clock.elapsed());
            sample.width = r.image.width(); sample.height = r.image.height();
            sample.wrongResolution |= sample.width != expectedWidth || sample.height != expectedHeight;
        });
        QObject::connect(videos[i], &VideoWidget::framePresented, &window, [&, i](quint64) {
            samples[i].painted.append(clock.elapsed());
        });
        QObject::connect(channels[i], &Channel::stateChanged, &window,
                         [&, i](MjpegStream::State state, const QString &detail) {
            samples[i].states.append(QJsonObject{{"ms", clock.elapsed()}, {"state", int(state)}, {"detail", detail}});
            samples[i].reconnect |= state == MjpegStream::State::Reconnecting;
        });
        QObject::connect(channels[i], &Channel::activeStreamChanged, &window, [&, i](const QUrl &url) {
            samples[i].transports.append(QJsonObject{{"ms", clock.elapsed()}, {"url", url.toString()}});
        });
        QObject::connect(channels[i], &Channel::videoProfileApplied, &window, [&, i](int profile) {
            samples[i].confirmations.append(QJsonObject{{"ms", clock.elapsed()}, {"profile", profile}});
        });
    }
    QObject::connect(&runtime, &CameraRuntime::streamReady, &window, &MainWindow::openChannel);
    QObject::connect(&runtime, &CameraRuntime::streamUnavailable, &window, &MainWindow::setChannelUnavailable);
    window.show();
    window.raise();
    window.activateWindow();
    visibilityTimer.start();
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
        const auto preset = VideoProfile::preset(window.videoProfile());
        const int targetFps = preset.fps;
        const int targetGapMs = targetFps == 25 ? 100 : 250;
        bool simultaneous = true, strict = true, targetStable = true;
        for (int i = 0; i < cameraCount; ++i) {
            const auto &sample = samples[i];
            const QJsonObject decoded = summarize(sample.decoded), painted = summarize(sample.painted);
            const bool resolution = !sample.wrongResolution && sample.width == expectedWidth && sample.height == expectedHeight;
            const bool display = !visibilityInterrupted && resolution
                                 && painted["min_fps_after_warmup"].toInt() >= 1 && !sample.reconnect;
            const bool performance = display && painted["min_fps_after_warmup"].toInt() >= 20 &&
                                     painted["max_gap_ms_after_warmup"].toInteger() <= 150;
            bool confirmed = false;
            for (const auto &confirmation : sample.confirmations)
                confirmed |= confirmation.toObject()["profile"].toInt() == window.videoProfile();
            const bool targetPerformance = display && confirmed
                && painted["min_fps_after_warmup"].toInt() >= targetFps
                && painted["max_gap_ms_after_warmup"].toInteger() <= targetGapMs;
            simultaneous &= display; strict &= performance;
            targetStable &= targetPerformance;
            reports.append(QJsonObject{{"id", i + 1}, {"url", urls[i].toString()},
                {"rotate180", videos[i]->rotate180()}, {"width", sample.width}, {"height", sample.height},
                {"wrong_resolution_observed", sample.wrongResolution}, {"reconnect_observed", sample.reconnect},
                {"decoded", decoded}, {"painted", painted}, {"states", sample.states},
                {"transports", sample.transports},
                {"profile_confirmations", sample.confirmations}, {"profile_confirmed", confirmed},
                {"display_passed", display}, {"strict_20fps_passed", performance},
                {"target_fps", targetFps}, {"target_stability_passed", targetPerformance}});
        }
        const bool missingClosed = !channels[1]->isOpen() && samples[1].decoded.isEmpty();
        const bool connectedPassed = oneConnected
            ? reports[0].toObject()["display_passed"].toBool() && missingClosed : simultaneous;
        const QJsonObject report{{"duration_s", seconds}, {"warmup_s", warmupMs / 1000},
            {"paint_measurement_valid", !visibilityInterrupted}, {"visibility", visibility},
            {"settings_path", settingsPath}, {"required_active_camera_count", oneConnected ? 1 : cameraCount},
            {"expected_missing_channel_closed", oneConnected && missingClosed},
            {"connected_camera_display_passed", connectedPassed},
            {"criteria", QJsonObject{{"functional_min_new_paints_each_second_per_camera", 1},
                {"strict_min_new_paints_each_second_per_camera", 20}, {"strict_max_gap_ms", 150},
                {"required_camera_count", oneConnected ? 1 : cameraCount}, {"required_width", expectedWidth}, {"required_height", expectedHeight}}},
            {"channels", reports}, {"simultaneous_display_passed", simultaneous}, {"strict_20fps_passed", strict},
            {"target_stability_passed", targetStable}, {"target_fps", targetFps}, {"target_max_gap_ms", targetGapMs},
            {"profile", videoProfileKey(window.videoProfile())},
            {"jpeg_minimum_quality", preset.minimumQuality}, {"jpeg_maximum_quality", preset.maximumQuality}};
        QFile file(output);
        if (!file.open(QIODevice::WriteOnly)) { app.exit(3); return; }
        file.write(QJsonDocument(report).toJson(QJsonDocument::Indented)); file.close();
        window.grab().save(output + ".png");
        for (auto *channel : channels) if (channel) channel->close();
#ifdef BASELINE_DIAGNOSTIC
        app.exit(targetStable ? 0 : 1);
#else
        app.exit(connectedPassed ? 0 : 1);
#endif
    });
    return app.exec();
}
