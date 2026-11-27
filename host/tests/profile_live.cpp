// Real three-camera VGA/QVGA display and button-switch acceptance.
#include <QApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPushButton>
#include <QTimer>
#include <QWindow>
#include <array>
#include <cstdio>
#include "cameraruntime.h"
#include "cameraconfigpath.h"
#include "mainwindow.h"
#include "videoprofile.h"
#include "videowidget.h"

struct Sample {
    QVector<qint64> decoded, painted;
    QJsonArray confirmations, states, transports;
    bool wrongResolution = false;
};

int main(int argc, char **argv)
{
    qInstallMessageHandler([](QtMsgType, const QMessageLogContext &, const QString &message) {
        const auto text = message.toUtf8();
        std::fprintf(stderr, "%s\n", text.constData());
        std::fflush(stderr);
    });
    QApplication app(argc, argv);
    if (argc != 3 && argc != 4) return 2; // optional "svga": third phase tests higher resolution
    const int seconds = app.arguments()[1].toInt();
    if (seconds < 20 || seconds > 600) return 2;
    const QString output = app.arguments()[2];
    CameraRuntime runtime(cameraSettingsPath());
    MainWindow window;
    window.setWindowFlag(Qt::WindowStaysOnTopHint);
    window.setVideoProfile(VideoProfile::Vga12);
    for (const auto profile : {VideoProfile::Vga12, VideoProfile::Qvga25, VideoProfile::Svga})
        window.findChild<QPushButton *>(QStringLiteral("profile-") + videoProfileKey(profile))->setEnabled(false);
    std::array<std::array<Sample, 3>, 3> samples;
    std::array<int, 3> profiles{VideoProfile::Vga12, VideoProfile::Qvga25, VideoProfile::Vga12};
    if (argc == 4 && app.arguments()[3] == QStringLiteral("svga")) profiles[2] = VideoProfile::Svga;
    int phase = 0;
    bool visible = true;
    QJsonArray guiStalls;
    QElapsedTimer clock;
    clock.start();
    QTimer guiHeartbeat;
    guiHeartbeat.setInterval(10);
    guiHeartbeat.setTimerType(Qt::PreciseTimer);
    qint64 lastGuiTick = 0;
    QObject::connect(&guiHeartbeat, &QTimer::timeout, &window, [&] {
        const qint64 now = clock.elapsed();
        if (now - lastGuiTick > 40)
            guiStalls.append(QJsonObject{{"ms", now}, {"gap_ms", now - lastGuiTick}});
        lastGuiTick = now;
    });
    guiHeartbeat.start();
    std::array<QFile, 3> serial;
    for (int i = 0; i < 3; ++i) {
        serial[i].setFileName(output + QStringLiteral(".channel%1.serial.log").arg(i + 1));
        if (!serial[i].open(QIODevice::WriteOnly)) return 3;
        auto *video = window.findChild<VideoWidget *>(QStringLiteral("video-%1").arg(i + 1));
        if (!video) return 2;
        QObject::connect(video, &VideoWidget::framePresented, &window, [&, i] {
            samples[phase][i].painted.append(clock.elapsed() - phase * seconds * 1000LL);
        });
    }
    for (auto *channel : window.findChildren<Channel *>()) {
        const int i = channel->id() - 1;
        QObject::connect(channel, &Channel::resultReady, &window, [&, i](const ChannelResult &result) {
            auto &sample = samples[phase][i];
            const auto preset = VideoProfile::preset(profiles[phase]);
            sample.decoded.append(clock.elapsed() - phase * seconds * 1000LL);
            if (clock.elapsed() - phase * seconds * 1000LL > 5000)
                sample.wrongResolution |= result.image.size() != QSize(preset.width, preset.height);
        });
        QObject::connect(channel, &Channel::videoProfileApplied, &window, [&, i](int profile) {
            samples[phase][i].confirmations.append(QJsonObject{
                {"ms", clock.elapsed() - phase * seconds * 1000LL}, {"profile", profile}});
        });
        QObject::connect(channel, &Channel::activeStreamChanged, &window, [&, i](const QUrl &url) {
            samples[phase][i].transports.append(url.toString());
        });
        QObject::connect(channel, &Channel::stateChanged, &window, [&, i](MjpegStream::State state, const QString &detail) {
            samples[phase][i].states.append(QJsonObject{
                {"ms", clock.elapsed() - phase * seconds * 1000LL}, {"state", int(state)}, {"detail", detail}});
        });
    }
    QObject::connect(&runtime, &CameraRuntime::serialDataReceived, &window, [&](int i, const QByteArray &data) {
        serial[i].write(data); serial[i].flush();
    });
    QObject::connect(&runtime, &CameraRuntime::streamReady, &window, &MainWindow::openChannel);
    QObject::connect(&runtime, &CameraRuntime::streamUnavailable, &window, &MainWindow::setChannelUnavailable);
    window.show();
    window.raise();
    window.activateWindow();
    runtime.start();
    QTimer visibility;
    visibility.setInterval(250);
    QObject::connect(&visibility, &QTimer::timeout, &window, [&] {
        if (clock.elapsed() - phase * seconds * 1000LL >= 5000)
            visible &= window.isVisible() && !window.isMinimized()
                       && window.windowHandle() && window.windowHandle()->isExposed();
    });
    visibility.start();
    for (int next = 1; next < 3; ++next) {
        QTimer::singleShot(next * seconds * 1000, Qt::PreciseTimer, &window, [&, next] {
            window.grab().save(output + QStringLiteral(".phase%1.png").arg(phase + 1));
            phase = next;
            auto *button = window.findChild<QPushButton *>(QStringLiteral("profile-") + videoProfileKey(profiles[phase]));
            button->setEnabled(true);
            button->click();
            button->setEnabled(false);
        });
    }
    QTimer::singleShot(3 * seconds * 1000, Qt::PreciseTimer, &window, [&] {
        window.grab().save(output + QStringLiteral(".phase3.png"));
        QJsonArray reports;
        bool allStable = visible, allSwitched = true;
        const auto summarize = [seconds](const QVector<qint64> &times) {
            QVector<int> bins(seconds, 0);
            qint64 previous = 5000, gap = 0;
            for (qint64 time : times) {
                if (time >= 0 && time < seconds * 1000LL) ++bins[int(time / 1000)];
                if (time >= 5000 && time < seconds * 1000LL) {
                    gap = qMax(gap, time - previous);
                    previous = time;
                }
            }
            gap = qMax(gap, seconds * 1000LL - previous);
            int minimum = 100000, total = 0, zeros = 0;
            QJsonArray counts, raw;
            for (int i = 0; i < bins.size(); ++i) {
                counts.append(bins[i]);
                if (i >= 5) {
                    minimum = qMin(minimum, bins[i]); total += bins[i];
                    if (bins[i] == 0) ++zeros;
                }
            }
            for (qint64 time : times) raw.append(time);
            return QJsonObject{{"per_second", counts}, {"times_ms", raw}, {"frames", times.size()},
                {"fps_after_warmup", double(total) / (seconds - 5)}, {"min_fps_after_warmup", minimum},
                {"zero_seconds_after_warmup", zeros}, {"max_gap_ms_after_warmup", gap},
                {"first_ms", times.isEmpty() ? -1 : times.first()}};
        };
        for (int p = 0; p < 3; ++p) {
            const auto preset = VideoProfile::preset(profiles[p]);
            QJsonArray channels;
            for (int i = 0; i < 3; ++i) {
                const auto &sample = samples[p][i];
                const auto decode = summarize(sample.decoded), paint = summarize(sample.painted);
                bool udpOnly = !sample.transports.isEmpty();
                for (const auto &transport : sample.transports)
                    udpOnly &= QUrl(transport.toString()).scheme() == QStringLiteral("udp");
                bool confirmed = false;
                for (const auto &confirmation : sample.confirmations)
                    confirmed |= confirmation.toObject()["profile"].toInt() == profiles[p];
                const bool switched = udpOnly && confirmed && !sample.wrongResolution && !sample.painted.isEmpty();
                bool reconnect = false;
                for (const auto &event : sample.states) {
                    const auto state = event.toObject();
                    reconnect |= state["ms"].toInteger() >= 5000
                        && state["state"].toInt() == int(MjpegStream::State::Reconnecting);
                }
                const int minimumFps = profiles[p] == VideoProfile::Qvga25 ? 20 : 12;
                const int maximumGap = profiles[p] == VideoProfile::Qvga25 ? 150 : 250;
                const bool stable = visible && switched
                    && !reconnect
                    && paint["min_fps_after_warmup"].toInt() >= minimumFps
                    && paint["max_gap_ms_after_warmup"].toInteger() <= maximumGap;
                allSwitched &= switched;
                allStable &= stable;
                channels.append(QJsonObject{{"id", i + 1}, {"decoded", decode}, {"painted", paint},
                    {"confirmations", sample.confirmations}, {"states", sample.states},
                    {"transports", sample.transports}, {"wrong_resolution", sample.wrongResolution},
                    {"reconnect_after_warmup", reconnect},
                    {"minimum_required_fps", minimumFps}, {"maximum_allowed_gap_ms", maximumGap},
                    {"switch_passed", switched}, {"stability_passed", stable}});
            }
            reports.append(QJsonObject{{"profile", videoProfileKey(profiles[p])}, {"width", preset.width},
                {"height", preset.height}, {"target_fps", preset.fps},
                {"source_interval_ms", int(VideoProfile::sendIntervalMs(profiles[p]))},
                {"channels", channels}});
        }
        QFile file(output);
        if (!file.open(QIODevice::WriteOnly)) { app.exit(3); return; }
        file.write(QJsonDocument(QJsonObject{{"seconds_per_mode", seconds}, {"warmup_seconds", 5},
            {"paint_measurement_valid", visible}, {"all_three_switch_passed", allSwitched},
            {"gui_stalls_over_40ms", guiStalls},
            {"target_stability_passed", allStable}, {"phases", reports}}).toJson(QJsonDocument::Indented));
        file.close();
        for (auto *channel : window.findChildren<Channel *>()) channel->close();
        app.exit(allStable ? 0 : 1);
    });
    return app.exec();
}
