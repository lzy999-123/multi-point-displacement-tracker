#include <QtTest>
#include <QComboBox>
#include <QDir>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSettings>
#include <QPainter>
#include <QNetworkDatagram>
#include <QUdpSocket>
#include <QTemporaryDir>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "mainwindow.h"
#include "videowidget.h"
#include "videoprofile.h"
#include "yolodetector.h"

namespace {
cv::Mat scene(bool target = true, bool background = true, int offset = 0)
{
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
    if (target) cv::rectangle(image, cv::Rect(100 + offset, 170, 60, 140), cv::Scalar(220, 220, 220), -1);
    if (background) cv::rectangle(image, cv::Rect(350, 100, 180, 240), cv::Scalar(220, 220, 220), -1);
    return image;
}

void stopDiscovery(MainWindow &window)
{
    for (auto *button : window.findChildren<QPushButton *>())
        if (button->text() == QStringLiteral("停止查找")) button->click();
}

void prepare(MainWindow &window)
{
    stopDiscovery(window);
    for (int i = 0; i < 3; ++i)
        window.openChannel(i, QUrl(QStringLiteral("udp://127.0.0.%1:59999").arg(i + 1)));
    QTest::qWait(40);
}

ChannelResult normal(Channel *channel)
{
    ChannelResult result;
    result.image = QImage(320, 240, QImage::Format_RGB888);
    result.image.fill(QColor(60, 60, 60));
    QPainter painter(&result.image);
    painter.fillRect(QRect(50, 85, 30, 70), QColor(220, 220, 220));
    painter.end();
    Detection detection;
    detection.valid = true;
    detection.angleReliable = true;
    detection.angle = -90;
    detection.area = 2100;
    detection.box = cv::Rect(50, 85, 30, 70);
    detection.center = cv::Point2d(65, 120);
    result.hasTarget = true;
    result.detections.append(detection);
    result.baselineDetection = detection;
    result.revision = channel->revision();
    result.observedAtMs = monitorNowMs();
    result.baselineCalibrated = true;
    result.anomaly.type = AnomalyType::Normal;
    result.anomaly.angleReliable = true;
    result.anomaly.badgeText = QStringLiteral("图像变化在阈值内");
    return result;
}

QString state(MainWindow &window)
{
    return window.findChild<QLabel *>("fusion-status")->property("monitorState").toString();
}

void injectNormal(MainWindow &window)
{
    for (auto *channel : window.findChildren<Channel *>()) {
        channel->stateChanged(MjpegStream::State::Streaming, QStringLiteral("software test input"));
        channel->resultReady(normal(channel));
    }
}

void screenshot(MainWindow &window, const QString &name)
{
    const QString directory = qEnvironmentVariable("MONITOR_TEST_OUTPUT");
    if (directory.isEmpty()) return;
    QDir().mkpath(directory);
    window.resize(1320, 880);
    window.ensurePolished();
    window.grab().save(directory + "/" + name + ".png");
}
}

class MonitorTest : public QObject
{
    Q_OBJECT
private slots:
    void requiresExplicitSelection() {
        AutoContourDetector detector;
        QVERIFY(detector.detect(scene()).isEmpty());
        detector.pickTarget(20, 20);
        QVERIFY(detector.detect(scene()).isEmpty());
        QVERIFY(detector.detect(scene()).isEmpty());
    }
    void clickedIdentitySurvivesIdenticalFrames() {
        AutoContourDetector detector;
        detector.pickTarget(130, 240);
        for (int i = 0; i < 15; ++i) {
            const auto found = detector.detect(scene());
            QCOMPARE(found.size(), 1);
            QVERIFY(found.first().center.x < 200);
        }
    }
    void disappearanceDoesNotAdoptBackground() {
        AutoContourDetector detector;
        detector.pickTarget(130, 240);
        const auto selected = detector.detect(scene());
        QVERIFY(!selected.isEmpty());
        BaselineState baseline;
        baseline.calibrate(selected.first());
        for (int i = 0; i < 10; ++i) {
            QVERIFY(detector.detect(scene(false)).isEmpty());
            QCOMPARE(baseline.evaluate(Detection{}, {}).type, AnomalyType::Lost);
        }
        QVERIFY(detector.detect(scene(false, false)).isEmpty());
        QVERIFY(detector.detect(scene(false)).isEmpty());
        // 同一位置重新出现的同形状目标可以关联；不是证明语义身份。
        QVERIFY(!detector.detect(scene()).isEmpty());
    }
    void movementStillProducesDisplacement() {
        AutoContourDetector detector;
        detector.pickTarget(130, 240);
        BaselineState baseline;
        baseline.calibrate(detector.detect(scene()).first());
        const auto moved = detector.detect(scene(true, true, 25));
        QCOMPARE(moved.size(), 1);
        const auto metric = baseline.evaluate(moved.first(), {});
        QCOMPARE(metric.type, AnomalyType::Displaced);
        QVERIFY(std::abs(metric.displacement - 25.0) < 1.0);
    }
    void explicitReselectionAndBlankClickClearOldIdentity() {
        AutoContourDetector detector;
        detector.pickTarget(130, 240);
        QVERIFY(detector.detect(scene()).first().center.x < 200);
        detector.pickTarget(440, 220);
        QVERIFY(detector.detect(scene()).first().center.x > 400);
        detector.pickTarget(20, 20);
        QVERIFY(detector.detect(scene()).isEmpty());
        QVERIFY(detector.detect(scene()).isEmpty());
    }
    void nearSymmetricShapesDoNotRaiseTilt() {
        AutoContourDetector detector;
        auto a = scene(false, false), b = a.clone();
        cv::rectangle(a, cv::Rect(270, 190, 100, 101), cv::Scalar(220, 220, 220), -1);
        cv::rectangle(b, cv::Rect(270, 190, 101, 100), cv::Scalar(220, 220, 220), -1);
        detector.pickTarget(320, 240);
        const auto first = detector.detect(a), second = detector.detect(b);
        QCOMPARE(first.size(), 1);
        QCOMPARE(second.size(), 1);
        QVERIFY(!first.first().angleReliable);
        QVERIFY(!second.first().angleReliable);
        BaselineState baseline;
        baseline.calibrate(first.first());
        const auto metric = baseline.evaluate(second.first(), {});
        QVERIFY(!metric.angleReliable);
        QCOMPARE(metric.type, AnomalyType::Normal);
    }
    void elongatedTargetStillMeasuresImageRotation() {
        AutoContourDetector detector;
        auto a = scene(true, false), b = scene(false, false);
        cv::Point2f vertices[4];
        cv::RotatedRect(cv::Point2f(130, 240), cv::Size2f(60, 140), 15).points(vertices);
        std::vector<cv::Point> polygon;
        for (auto point : vertices) polygon.emplace_back(cvRound(point.x), cvRound(point.y));
        cv::fillConvexPoly(b, polygon, cv::Scalar(220, 220, 220));
        detector.pickTarget(130, 240);
        BaselineState baseline;
        baseline.calibrate(detector.detect(a).first());
        const auto rotated = detector.detect(b);
        QCOMPARE(rotated.size(), 1);
        const auto metric = baseline.evaluate(rotated.first(), {});
        QVERIFY(metric.angleReliable);
        QCOMPARE(metric.type, AnomalyType::Tilted);
        QVERIFY(std::abs(metric.deltaAngle - 15.0) < 2.0);
    }
    void alarmAndRecoveryUseElapsedTime() {
        AnomalyDebouncer filter;
        AnomalyMetrics safe; safe.type = AnomalyType::Normal;
        QCOMPARE(filter.update(safe, 0).type, AnomalyType::Pending);
        QCOMPARE(filter.update(safe, 1000).type, AnomalyType::Normal);
        AnomalyMetrics lost; lost.type = AnomalyType::Lost;
        QCOMPARE(filter.update(lost, 1100).type, AnomalyType::Pending);
        QCOMPARE(filter.update(lost, 1599).type, AnomalyType::Pending);
        QCOMPARE(filter.update(lost, 1600).type, AnomalyType::Lost);
        QCOMPARE(filter.update(safe, 1700).type, AnomalyType::Lost);
        QCOMPARE(filter.update(safe, 2699).type, AnomalyType::Lost);
        QCOMPARE(filter.update(safe, 2700).type, AnomalyType::Normal);
        filter.reset();
        QCOMPARE(filter.update(lost, 10000).type, AnomalyType::Pending);
    }
    void transientFailureNeverReportsNormalWhilePending() {
        AnomalyDebouncer filter;
        AnomalyMetrics safe; safe.type = AnomalyType::Normal;
        filter.update(safe, 0); filter.update(safe, 1000);
        AnomalyMetrics change; change.type = AnomalyType::Displaced;
        QCOMPARE(filter.update(change, 1100).type, AnomalyType::Pending);
        QCOMPARE(filter.update(safe, 1200).type, AnomalyType::Normal);
        QCOMPARE(filter.update(change, 1300).type, AnomalyType::Pending);
        QCOMPARE(filter.update(change, 1799).type, AnomalyType::Pending);
    }
    void partialLossAndReconnectionCannotReportNormal() {
        QTemporaryDir directory;
        MainWindow window(nullptr, directory.filePath("camera.ini"));
        prepare(window);
        injectNormal(window);
        QCOMPARE(state(window), QStringLiteral("normal"));
        screenshot(window, "normal");
        const auto channels = window.findChildren<Channel *>();
        auto lost = normal(channels[0]); lost.anomaly.type = AnomalyType::Lost;
        lost.hasTarget = false; lost.detections.clear(); lost.anomaly.badgeText = QStringLiteral("目标不可见");
        channels[0]->resultReady(lost);
        QCOMPARE(state(window), QStringLiteral("lost"));
        lost = normal(channels[1]); lost.anomaly.type = AnomalyType::Lost;
        lost.hasTarget = false; lost.detections.clear(); lost.anomaly.badgeText = QStringLiteral("目标不可见");
        channels[1]->resultReady(lost);
        QCOMPARE(state(window), QStringLiteral("lost"));
        screenshot(window, "partial-loss");
        injectNormal(window);
        for (auto *channel : channels)
            channel->stateChanged(MjpegStream::State::Reconnecting, QStringLiteral("test outage"));
        QCOMPARE(state(window), QStringLiteral("unavailable"));
        screenshot(window, "reconnecting");
    }
    void freshnessExpiresWithoutAnotherFrameOrStateSignal() {
        QTemporaryDir directory;
        MainWindow window(nullptr, directory.filePath("camera.ini"));
        prepare(window);
        injectNormal(window);
        QCOMPARE(state(window), QStringLiteral("normal"));
        QTRY_COMPARE_WITH_TIMEOUT(state(window), QStringLiteral("unavailable"), 1400);
    }
    void incompleteAndUncalibratedCoverageCannotReportNormal() {
        QTemporaryDir directory;
        MainWindow window(nullptr, directory.filePath("camera.ini"));
        prepare(window);
        const auto channels = window.findChildren<Channel *>();
        channels[0]->resultReady(normal(channels[0]));
        QCOMPARE(state(window), QStringLiteral("unavailable"));
        injectNormal(window);
        auto raw = normal(channels[2]);
        raw.baselineCalibrated = false;
        raw.anomaly.type = AnomalyType::Uncalibrated;
        channels[2]->resultReady(raw);
        QCOMPARE(state(window), QStringLiteral("uncalibrated"));
    }
    void oldSessionAndStaleFramesCannotRestoreNormal() {
        QTemporaryDir directory;
        MainWindow window(nullptr, directory.filePath("camera.ini"));
        prepare(window);
        injectNormal(window);
        auto *channel = window.findChildren<Channel *>().first();
        const auto old = normal(channel);
        window.openChannel(channel->id() - 1, channel->url());
        channel->resultReady(old);
        QVERIFY(state(window) != QStringLiteral("normal"));
        auto stale = normal(channel);
        stale.observedAtMs -= kMonitorFreshnessMs + 100;
        channel->resultReady(stale);
        QVERIFY(state(window) != QStringLiteral("normal"));
    }
    void calibrationRequiresARecentChosenTarget() {
        auto pending = std::make_shared<std::atomic<int>>(0);
        ChannelWorker worker(pending);
        QSignalSpy spy(&worker, &ChannelWorker::calibrationFinished);
        worker.calibrateBaseline();
        QCOMPARE(spy.size(), 1);
        QVERIFY(!spy.first()[1].toBool());
    }
    void realChannelSelectionCalibrationLossAndStaleRejection() {
        QUdpSocket camera;
        QVERIFY(camera.bind(QHostAddress::LocalHost, 0));
        QHostAddress receiver;
        quint16 port = 0, sequence = 0;
        bool visible = true;
        auto send = [&] {
            if (!port) return;
            auto full = scene(visible);
            cv::Mat image;
            cv::resize(full, image, cv::Size(320, 240));
            std::vector<uchar> bytes;
            QVERIFY(cv::imencode(".jpg", image, bytes));
            const QByteArray jpeg(reinterpret_cast<const char *>(bytes.data()), int(bytes.size()));
            const int count = (jpeg.size() + 1399) / 1400;
            ++sequence;
            for (int i = 0; i < count; ++i) {
                const auto payload = jpeg.mid(i * 1400, 1400);
                const VidPacketHeader header{0x31444956, 1, 3, sequence, quint8(i),
                    quint8(count), quint16(payload.size()), quint32(jpeg.size())};
                camera.writeDatagram(QByteArray(reinterpret_cast<const char *>(&header), sizeof(header))
                                     + payload, receiver, port);
            }
        };
        connect(&camera, &QUdpSocket::readyRead, &camera, [&] {
            while (camera.hasPendingDatagrams()) {
                const auto datagram = camera.receiveDatagram();
                if (!datagram.data().startsWith("VREQA3")) continue;
                receiver = datagram.senderAddress();
                port = datagram.senderPort();
            }
        });
        QTimer timer;
        connect(&timer, &QTimer::timeout, &camera, send);
        timer.start(100);
        Channel channel(1);
        channel.setDetectorMode(ChannelWorker::AutoContour);
        ChannelResult latest;
        connect(&channel, &Channel::resultReady, &camera, [&](const ChannelResult &result) {
            if (result.revision == channel.revision()) latest = result;
            channel.acknowledgeResult();
        });
        channel.open(withVideoProfile(QUrl(QStringLiteral("udp://127.0.0.1:%1").arg(camera.localPort())),
                                      VideoProfile::Baseline10));
        QTRY_VERIFY_WITH_TIMEOUT(!latest.image.isNull(), 2000);
        QVERIFY(!latest.hasTarget);
        channel.pickTarget(65, 120);
        QTRY_VERIFY_WITH_TIMEOUT(latest.hasTarget && latest.revision == channel.revision(), 1500);
        QSignalSpy calibration(&channel, &Channel::calibrationFinished);
        channel.calibrateBaseline();
        QTRY_COMPARE(calibration.size(), 1);
        QVERIFY(calibration.first()[1].toBool());
        QTRY_VERIFY_WITH_TIMEOUT(latest.baselineCalibrated && latest.anomaly.type == AnomalyType::Normal, 1800);
        visible = false;
        QTRY_VERIFY_WITH_TIMEOUT(latest.anomaly.type == AnomalyType::Lost, 1500);
        QVERIFY(!latest.hasTarget);
        timer.stop();
        QTest::qWait(kMonitorFreshnessMs + 100);
        channel.calibrateBaseline();
        QTRY_COMPARE(calibration.size(), 2);
        QVERIFY(!calibration.last()[1].toBool());
        visible = true;
        timer.start(100);
        channel.open(channel.url());
        QTRY_VERIFY_WITH_TIMEOUT(latest.revision == channel.revision(), 1500);
        QVERIFY(!latest.baselineCalibrated);
        QVERIFY(!latest.hasTarget);
    }
    void productionProfilesKeepSelectionAndExposeSwitchButtons() {
        QTemporaryDir directory;
        const QString path = directory.filePath("camera.ini");
        { QSettings settings(path, QSettings::IniFormat); settings.setValue("video/profile", "qvga25"); }
        MainWindow window(nullptr, path);
        stopDiscovery(window);
        QCOMPARE(window.videoProfile(), int(VideoProfile::Qvga25));
        const auto preset = VideoProfile::preset(window.videoProfile());
        QCOMPARE(preset.fps, 0);
        QCOMPARE(preset.width, 320);
        QVERIFY(window.findChild<QPushButton *>("profile-qvga25")->isChecked());
        QVERIFY(window.findChild<QPushButton *>("profile-vga12"));
        QVERIFY(window.findChild<QPushButton *>("profile-svga"));
        const auto *v1 = window.findChild<VideoWidget *>("video-1");
        const auto *v2 = window.findChild<VideoWidget *>("video-2");
        const auto *v3 = window.findChild<VideoWidget *>("video-3");
        QVERIFY(v1 && v2 && v3);
        QCOMPARE(v1->size(), QSize(480, 360));
        QCOMPARE(v2->size(), v1->size());
        QCOMPARE(v3->size(), v1->size());
        QVERIFY(v1->parentWidget()->findChildren<QLineEdit *>().isEmpty());
        QVERIFY(v2->parentWidget()->findChildren<QLineEdit *>().isEmpty());
        QVERIFY(v3->parentWidget()->findChildren<QLineEdit *>().isEmpty());
    }
    void roiSelectionTracksStillMoveScaleTiltAndDisappearance() {
        auto makeArtifactScene = [](bool present, int cx, int cy, double scale, double angleDeg) {
            cv::Mat img(480, 640, CV_8UC3, cv::Scalar(48, 52, 56));
            // 固定背景干扰物，确保不会误接管背景
            cv::rectangle(img, cv::Rect(480, 40, 120, 140), cv::Scalar(160, 160, 160), -1);
            if (present) {
                const cv::Size2f sz(float(56.0 * scale), float(124.0 * scale));
                cv::RotatedRect rr(cv::Point2f(float(cx), float(cy)), sz, float(angleDeg));
                cv::Point2f pts[4];
                rr.points(pts);
                std::vector<cv::Point> poly;
                for (auto p : pts) poly.emplace_back(cvRound(p.x), cvRound(p.y));
                cv::fillConvexPoly(img, poly, cv::Scalar(40, 130, 210));
                // 内部花纹模拟真实博物馆展品纹理
                cv::RotatedRect inner(cv::Point2f(float(cx), float(cy)),
                                      cv::Size2f(sz.width * 0.45f, sz.height * 0.55f), float(angleDeg));
                inner.points(pts);
                std::vector<cv::Point> innerPoly;
                for (auto p : pts) innerPoly.emplace_back(cvRound(p.x), cvRound(p.y));
                cv::fillConvexPoly(img, innerPoly, cv::Scalar(195, 220, 240));
            }
            return img;
        };

        AutoContourDetector detector;
        const cv::Mat baseFrame = makeArtifactScene(true, 180, 240, 1.0, 0.0);
        detector.selectRoi(cv::Rect(140, 165, 80, 150));
        const auto init = detector.detect(baseFrame);
        QCOMPARE(init.size(), 1);
        QVERIFY(init.first().valid);
        BaselineState baseline;
        baseline.calibrate(init.first(), baseFrame);

        // 1. 静止无变化
        const auto still = detector.detect(baseFrame);
        QCOMPARE(still.size(), 1);
        QCOMPARE(baseline.evaluate(still.first(), {}).type, AnomalyType::Normal);

        // 2. 大幅度移动 (+120px，超出旧版 0.8*diag 门限)
        const auto moved = detector.detect(makeArtifactScene(true, 300, 240, 1.0, 0.0));
        QCOMPARE(moved.size(), 1);
        const auto moveMetric = baseline.evaluate(moved.first(), {});
        QCOMPARE(moveMetric.type, AnomalyType::Displaced);
        QVERIFY(std::abs(moveMetric.deltaX - 120.0) < 8.0);

        // 3. 目标变大（移近镜头）
        const auto larger = detector.detect(makeArtifactScene(true, 180, 240, 1.25, 0.0));
        QCOMPARE(larger.size(), 1);
        const auto largerMetric = baseline.evaluate(larger.first(), {});
        QCOMPARE(largerMetric.type, AnomalyType::ScaledLarger);
        QVERIFY(largerMetric.scaleRatio > 1.35);

        // 4. 目标变小（远离镜头）
        const auto smaller = detector.detect(makeArtifactScene(true, 180, 240, 0.78, 0.0));
        QCOMPARE(smaller.size(), 1);
        const auto smallerMetric = baseline.evaluate(smaller.first(), {});
        QCOMPARE(smallerMetric.type, AnomalyType::ScaledSmaller);
        QVERIFY(smallerMetric.scaleRatio < 0.75);

        // 5. 原地倾斜 35°
        const auto tilted = detector.detect(makeArtifactScene(true, 180, 240, 1.0, 35.0));
        QCOMPARE(tilted.size(), 1);
        const auto tiltMetric = baseline.evaluate(tilted.first(), {});
        QCOMPARE(tiltMetric.type, AnomalyType::Tilted);
        QVERIFY(tiltMetric.deltaAngle > 20.0);

        // 6. 移走后不接管背景，报告目标不可见
        QVERIFY(detector.detect(makeArtifactScene(false, 180, 240, 1.0, 0.0)).isEmpty());

        // 7. 放回原位后恢复正常
        const auto restored = detector.detect(baseFrame);
        QCOMPARE(restored.size(), 1);
        QCOMPARE(baseline.evaluate(restored.first(), {}).type, AnomalyType::Normal);
    }
    void yellowBottleDoesNotJumpToOtherColoredBottlesAndRedetects() {
        auto drawBottle = [](cv::Mat &img, int cx, int cy, const cv::Scalar &body, const cv::Scalar &cap) {
            cv::rectangle(img, cv::Rect(cx - 24, cy - 42, 48, 96), body, -1);
            cv::rectangle(img, cv::Rect(cx - 12, cy - 62, 24, 22), body, -1);
            cv::rectangle(img, cv::Rect(cx - 14, cy - 72, 28, 12), cap, -1);
            // 所有瓶子贴同样白色标签条，专门考验颜色身份门控与抗同形干扰能力
            cv::rectangle(img, cv::Rect(cx - 22, cy - 6, 44, 22), cv::Scalar(235, 240, 242), -1);
            cv::line(img, cv::Point(cx - 14, cy + 4), cv::Point(cx + 14, cy + 4), cv::Scalar(90, 90, 90), 2);
        };
        auto makeBottleScene = [&](bool hasYellow, int yx, int yy) {
            cv::Mat img(480, 640, CV_8UC3, cv::Scalar(120, 122, 125));
            // 蓝瓶和绿瓶固定在桌面，与黄瓶外形完全一致
            drawBottle(img, 320, 250, cv::Scalar(210, 115, 30), cv::Scalar(195, 95, 20));
            drawBottle(img, 500, 250, cv::Scalar(50, 180, 55), cv::Scalar(35, 160, 45));
            if (hasYellow)
                drawBottle(img, yx, yy, cv::Scalar(25, 205, 235), cv::Scalar(20, 185, 220));
            return img;
        };

        AutoContourDetector detector;
        const cv::Mat frame0 = makeBottleScene(true, 140, 250);
        detector.selectRoi(cv::Rect(108, 174, 64, 152));
        const auto init = detector.detect(frame0);
        QCOMPARE(init.size(), 1);
        QVERIFY(detector.hasActiveSelection());
        QVERIFY(std::abs(init.first().center.x - 140.0) < 10.0);

        // 1. 黄瓶向右移动靠近蓝瓶，应紧跟完整黄瓶（含中间白色标签），不偏到上半截也不跳到蓝瓶
        for (int cx : {165, 195, 225, 255}) {
            const auto tracked = detector.detect(makeBottleScene(true, cx, 250));
            QCOMPARE(tracked.size(), 1);
            QVERIFY2(std::abs(tracked.first().center.x - cx) < 16.0
                         && std::abs(tracked.first().center.y - 250.0) < 16.0,
                     qPrintable(QString("expected (%1,250) got (%2,%3)")
                                    .arg(cx).arg(tracked.first().center.x).arg(tracked.first().center.y)));
        }

        // 2. 黄瓶跨过蓝瓶出现在蓝绿瓶之间，应迅速跟到新位置
        const auto crossed = detector.detect(makeBottleScene(true, 410, 250));
        QCOMPARE(crossed.size(), 1);
        QVERIFY2(std::abs(crossed.first().center.x - 410.0) < 16.0
                     && std::abs(crossed.first().center.y - 250.0) < 16.0,
                 qPrintable(QString("expected (410,250) got (%1,%2)")
                                .arg(crossed.first().center.x).arg(crossed.first().center.y)));

        // 3. 黄瓶移出视野，桌面只剩外形完全相同的蓝瓶和绿瓶：必须报丢，绝不能串锁蓝/绿瓶
        for (int i = 0; i < 5; ++i) {
            const auto absent = detector.detect(makeBottleScene(false, 0, 0));
            QVERIFY2(absent.isEmpty(),
                     absent.isEmpty() ? "" : qPrintable(QString("falsely locked at x=%1").arg(absent.first().center.x)));
        }

        // 4. 黄瓶在全新远端位置 (570, 210) 重新出现：应全图自动重找锁定
        const auto recovered = detector.detect(makeBottleScene(true, 570, 210));
        QCOMPARE(recovered.size(), 1);
        QVERIFY2(std::abs(recovered.first().center.x - 570.0) < 16.0
                     && std::abs(recovered.first().center.y - 210.0) < 16.0,
                 qPrintable(QString("expected (570,210) got (%1,%2)")
                                .arg(recovered.first().center.x).arg(recovered.first().center.y)));

        // 5. 清除目标后停止跟踪
        detector.resetTracking();
        QVERIFY(!detector.hasActiveSelection());
        QVERIFY(detector.detect(frame0).isEmpty());
    }
    void crossCameraSignatureSyncLocksDifferentAngleLightingAndRejectsOtherColors() {
        auto drawScaledBottle = [](cv::Mat &img, int cx, int cy, double scale,
                                   const cv::Scalar &body, const cv::Scalar &cap) {
            const int bw = int(std::lround(48 * scale));
            const int bh = int(std::lround(96 * scale));
            const int nw = int(std::lround(24 * scale));
            const int nh = int(std::lround(22 * scale));
            const int cw = int(std::lround(28 * scale));
            const int ch = int(std::lround(12 * scale));
            cv::rectangle(img, cv::Rect(cx - bw / 2, cy - int(std::lround(42 * scale)), bw, bh), body, -1);
            cv::rectangle(img, cv::Rect(cx - nw / 2, cy - int(std::lround(62 * scale)), nw, nh), body, -1);
            cv::rectangle(img, cv::Rect(cx - cw / 2, cy - int(std::lround(72 * scale)), cw, ch), cap, -1);
            cv::rectangle(img, cv::Rect(cx - int(std::lround(22 * scale)), cy - int(std::lround(6 * scale)),
                                        int(std::lround(44 * scale)), int(std::lround(22 * scale))),
                          cv::Scalar(232, 236, 240), -1);
        };

        // 机位 1：正面视角，框选黄色瓶子
        cv::Mat cam1(480, 640, CV_8UC3, cv::Scalar(120, 122, 125));
        drawScaledBottle(cam1, 150, 260, 1.0, cv::Scalar(25, 205, 235), cv::Scalar(20, 185, 220));
        drawScaledBottle(cam1, 340, 260, 1.0, cv::Scalar(210, 115, 30), cv::Scalar(195, 95, 20));
        drawScaledBottle(cam1, 510, 260, 1.0, cv::Scalar(50, 180, 55), cv::Scalar(35, 160, 45));

        AutoContourDetector det1, det2, det3;
        det1.selectRoi(cv::Rect(118, 182, 64, 154));
        const auto d1 = det1.detect(cam1);
        QCOMPARE(d1.size(), 1);
        const TargetSignature sig = det1.exportSignature(cam1);
        QVERIFY(sig.valid);
        QVERIFY(sig.isChromatic);

        // 广播签名给机位 2 和机位 3
        det2.adoptSignature(sig);
        det3.adoptSignature(sig);
        QVERIFY(det2.hasActiveSelection());
        QVERIFY(det3.hasActiveSelection());

        // 机位 2：近景侧视角（1.25x 放大 + 偏暗白平衡色偏 + 完全不同的位置，且有蓝/绿瓶干扰）
        auto makeCam2 = [&](int yx, int yy) {
            cv::Mat img(480, 640, CV_8UC3, cv::Scalar(105, 108, 112));
            drawScaledBottle(img, 140, 250, 1.25, cv::Scalar(195, 100, 28), cv::Scalar(180, 85, 20));
            drawScaledBottle(img, 300, 260, 1.25, cv::Scalar(42, 162, 48), cv::Scalar(32, 145, 38));
            drawScaledBottle(img, yx, yy, 1.25, cv::Scalar(18, 175, 205), cv::Scalar(15, 158, 190));
            return img;
        };
        const auto d2Init = det2.detect(makeCam2(470, 220));
        QCOMPARE(d2Init.size(), 1);
        QVERIFY2(std::abs(d2Init.first().center.x - 470.0) < 16.0
                     && std::abs(d2Init.first().center.y - 220.0) < 16.0,
                 qPrintable(QString("cam2 expected (470,220) got (%1,%2)")
                                .arg(d2Init.first().center.x).arg(d2Init.first().center.y)));

        // 机位 2 锁定后切入本地 CSRT，目标移动应顺滑跟住
        const auto d2Move = det2.detect(makeCam2(420, 230));
        QCOMPARE(d2Move.size(), 1);
        QVERIFY2(std::abs(d2Move.first().center.x - 420.0) < 16.0
                     && std::abs(d2Move.first().center.y - 230.0) < 16.0,
                 qPrintable(QString("cam2 move expected (420,230) got (%1,%2)")
                                .arg(d2Move.first().center.x).arg(d2Move.first().center.y)));

        // 机位 3：远景视角（0.72x 缩小），起初画面里只有蓝/绿瓶，必须保持搜索不误锁
        auto makeCam3 = [&](bool hasYellow, int yx, int yy) {
            cv::Mat img(480, 640, CV_8UC3, cv::Scalar(130, 128, 126));
            drawScaledBottle(img, 360, 230, 0.72, cv::Scalar(215, 120, 35), cv::Scalar(200, 100, 25));
            drawScaledBottle(img, 490, 240, 0.72, cv::Scalar(55, 185, 60), cv::Scalar(40, 165, 48));
            if (hasYellow)
                drawScaledBottle(img, yx, yy, 0.72, cv::Scalar(30, 210, 240), cv::Scalar(25, 190, 220));
            return img;
        };
        for (int i = 0; i < 3; ++i) {
            QVERIFY(det3.detect(makeCam3(false, 0, 0)).isEmpty());
        }
        // 黄瓶进入机位 3 画面后，自动捕捉并锁定
        const auto d3Found = det3.detect(makeCam3(true, 210, 190));
        QCOMPARE(d3Found.size(), 1);
        QVERIFY2(std::abs(d3Found.first().center.x - 210.0) < 16.0
                     && std::abs(d3Found.first().center.y - 190.0) < 16.0,
                 qPrintable(QString("cam3 expected (210,190) got (%1,%2)")
                                .arg(d3Found.first().center.x).arg(d3Found.first().center.y)));
    }
    void multiViewFusionDistinguishesSingleWarningFromMultiConfirmed() {
        QTemporaryDir directory;
        MainWindow window(nullptr, directory.filePath("camera.ini"));
        prepare(window);
        injectNormal(window);
        auto *statusLabel = window.findChild<QLabel *>("fusion-status");
        QCOMPARE(statusLabel->property("monitorState").toString(), QStringLiteral("normal"));
        QCOMPARE(statusLabel->property("monitorSeverity").toString(), QStringLiteral("normal"));

        const auto channels = window.findChildren<Channel *>();
        // 1 路目标不可见 -> 单视角预警
        auto lost1 = normal(channels[0]);
        lost1.anomaly.type = AnomalyType::Lost;
        lost1.hasTarget = false; lost1.detections.clear();
        lost1.anomaly.badgeText = QStringLiteral("目标不可见");
        channels[0]->resultReady(lost1);
        QCOMPARE(statusLabel->property("monitorState").toString(), QStringLiteral("lost"));
        QCOMPARE(statusLabel->property("monitorSeverity").toString(), QStringLiteral("single"));

        // 2 路目标不可见 -> 多视角确认告警
        auto lost2 = normal(channels[1]);
        lost2.anomaly.type = AnomalyType::Lost;
        lost2.hasTarget = false; lost2.detections.clear();
        lost2.anomaly.badgeText = QStringLiteral("目标不可见");
        channels[1]->resultReady(lost2);
        QCOMPARE(statusLabel->property("monitorState").toString(), QStringLiteral("lost"));
        QCOMPARE(statusLabel->property("monitorSeverity").toString(), QStringLiteral("confirmed"));

        // 恢复正常后，测试 1 路变大 -> 单视角提示；再加 1 路移动 -> 多视角确认空间移动
        injectNormal(window);
        auto bigger = normal(channels[0]);
        bigger.anomaly.type = AnomalyType::ScaledLarger;
        bigger.anomaly.badgeText = QStringLiteral("⚠️ 变大 +30%");
        channels[0]->resultReady(bigger);
        QCOMPARE(statusLabel->property("monitorState").toString(), QStringLiteral("changed"));
        QCOMPARE(statusLabel->property("monitorSeverity").toString(), QStringLiteral("single"));

        auto moved = normal(channels[1]);
        moved.anomaly.type = AnomalyType::Displaced;
        moved.anomaly.badgeText = QStringLiteral("⚠️ 移动 25.0px");
        channels[1]->resultReady(moved);
        QCOMPARE(statusLabel->property("monitorState").toString(), QStringLiteral("changed"));
        QCOMPARE(statusLabel->property("monitorSeverity").toString(), QStringLiteral("confirmed"));
        QVERIFY(statusLabel->text().contains(QStringLiteral("空间移动")));
    }
    void targetStateTrackerReportsEverMovedScaleAndTopple() {
        auto makeDet = [](double cx, double cy, int w, int h, double area, double angle, bool angleOk) {
            Detection d;
            d.valid = true;
            d.center = cv::Point2d(cx, cy);
            d.box = cv::Rect(int(std::lround(cx - w * 0.5)), int(std::lround(cy - h * 0.5)), w, h);
            d.area = area;
            d.aspectRatio = double(h) / std::max(1, w);
            d.angle = angle;
            d.angleReliable = angleOk;
            return d;
        };

        TargetStateTracker tracker(200);
        const MonitorThresholds th;
        const Detection base = makeDet(200, 240, 50, 140, 7000.0, 90.0, true);

        // 1. 首次捕获自动建立参考：静止、正常、未倾倒
        auto st = tracker.update(base, th, 1000);
        QVERIFY(st.active && st.visible);
        QCOMPARE(st.motion, TargetStatus::MotionState::Stationary);
        QCOMPARE(st.size, TargetStatus::SizeState::Normal);
        QCOMPARE(st.posture, TargetStatus::PostureState::Normal);
        QVERIFY(!st.hasEverMoved);
        QVERIFY(!st.hasEverToppled);

        // 2. 短暂 100ms 抖动不应误触发“曾移动过”
        const Detection moved = makeDet(245, 240, 50, 140, 7000.0, 90.0, true);
        st = tracker.update(moved, th, 1050);
        QCOMPARE(st.motion, TargetStatus::MotionState::Stationary);
        QVERIFY(!st.hasEverMoved);
        st = tracker.update(base, th, 1150);
        QVERIFY(!st.hasEverMoved);

        // 3. 持续移位 >= 200ms：进入已移动并记录 hasEverMoved
        tracker.update(moved, th, 1200);
        st = tracker.update(moved, th, 1450);
        QCOMPARE(st.motion, TargetStatus::MotionState::Displaced);
        QVERIFY(st.hasEverMoved);
        QVERIFY(st.maxDisplacement >= 44.0);
        QVERIFY(st.motionText.contains(QStringLiteral("已移动")));

        // 4. 移回原位持续 >= 200ms：输出“曾移动过(已回位)”
        tracker.update(base, th, 1500);
        st = tracker.update(base, th, 1750);
        QCOMPARE(st.motion, TargetStatus::MotionState::MovedReturned);
        QVERIFY(st.hasEverMoved);
        QVERIFY(st.motionText.contains(QStringLiteral("曾移动过")));
        QVERIFY(st.shortTag.contains(QStringLiteral("曾移动过")));

        // 5. 变大与变小
        const Detection larger = makeDet(200, 240, 62, 174, 10500.0, 90.0, true);
        tracker.update(larger, th, 1800);
        st = tracker.update(larger, th, 2050);
        QCOMPARE(st.size, TargetStatus::SizeState::Larger);
        QVERIFY(st.sizeText.contains(QStringLiteral("变大")));

        const Detection smaller = makeDet(200, 240, 38, 106, 4200.0, 90.0, true);
        tracker.update(smaller, th, 2100);
        st = tracker.update(smaller, th, 2350);
        QCOMPARE(st.size, TargetStatus::SizeState::Smaller);
        QVERIFY(st.sizeText.contains(QStringLiteral("变小")));
        // 纯等比缩小不能误报倾倒
        QCOMPARE(st.posture, TargetStatus::PostureState::Normal);

        // 6. 轻微倾斜 16° -> Tilted；大幅倾倒 84° + 移位 + 变大 -> 多维度并行输出
        const Detection tilted = makeDet(200, 240, 72, 136, 7000.0, 74.0, true);
        tracker.update(tilted, th, 2400);
        st = tracker.update(tilted, th, 2650);
        QCOMPARE(st.posture, TargetStatus::PostureState::Tilted);
        QVERIFY(!st.hasEverToppled);

        const Detection compoundTopple = makeDet(260, 240, 165, 58, 9600.0, 6.0, true);
        tracker.update(compoundTopple, th, 2700);
        st = tracker.update(compoundTopple, th, 2950);
        QCOMPARE(st.motion, TargetStatus::MotionState::Displaced);
        QCOMPARE(st.size, TargetStatus::SizeState::Larger);
        QCOMPARE(st.posture, TargetStatus::PostureState::Toppled);
        QVERIFY(st.hasEverToppled);
        QVERIFY(st.shortTag.contains(QStringLiteral("已倾倒")));
        QVERIFY(st.shortTag.contains(QStringLiteral("变大")));
        QVERIFY(st.shortTag.contains(QStringLiteral("已移动")));

        // 7. 朝向镜头倒下（高度塌陷而宽度不变、无可靠主轴角）也应判定倾倒
        tracker.setReference(base);
        QVERIFY(!tracker.update(base, th, 3000).hasEverMoved);
        const Detection foreshortened = makeDet(200, 240, 52, 68, 3536.0, 0.0, false);
        tracker.update(foreshortened, th, 3050);
        st = tracker.update(foreshortened, th, 3300);
        QCOMPARE(st.posture, TargetStatus::PostureState::Toppled);
        QVERIFY(st.toppledByAspect);
    }
    void labeledBottleAndGrayBoxTopple90DegTracking() {
        auto drawRotatedBottle = [](cv::Mat &img, int cx, int cy, double angleDeg,
                                    const cv::Scalar &body, const cv::Scalar &cap) {
            auto fillRelRect = [&](float rx, float ry, float rw, float rh, const cv::Scalar &color) {
                const double rad = angleDeg * 3.14159265358979323846 / 180.0;
                const double ca = std::cos(rad), sa = std::sin(rad);
                const float gx = float(cx + rx * ca - ry * sa);
                const float gy = float(cy + rx * sa + ry * ca);
                cv::RotatedRect rr(cv::Point2f(gx, gy), cv::Size2f(rw, rh), float(angleDeg));
                cv::Point2f pts[4];
                rr.points(pts);
                std::vector<cv::Point> poly;
                for (auto p : pts) poly.emplace_back(cvRound(p.x), cvRound(p.y));
                cv::fillConvexPoly(img, poly, color);
            };
            fillRelRect(0.0f, 6.0f, 48.0f, 96.0f, body);
            fillRelRect(0.0f, -51.0f, 24.0f, 22.0f, body);
            fillRelRect(0.0f, -66.0f, 28.0f, 12.0f, cap);
            fillRelRect(0.0f, 5.0f, 44.0f, 22.0f, cv::Scalar(235, 240, 242)); // 中间白标签
        };
        auto makeScene = [&](int yx, int yy, double angleDeg) {
            cv::Mat img(480, 640, CV_8UC3, cv::Scalar(120, 122, 125));
            drawRotatedBottle(img, 520, 260, 0.0, cv::Scalar(210, 115, 30), cv::Scalar(195, 95, 20));
            drawRotatedBottle(img, yx, yy, angleDeg, cv::Scalar(25, 205, 235), cv::Scalar(20, 185, 220));
            return img;
        };

        AutoContourDetector detector;
        TargetStateTracker tracker(0);
        const MonitorThresholds th;

        // 1. 立放黄瓶
        detector.selectRoi(cv::Rect(148, 174, 64, 152));
        const auto d0 = detector.detect(makeScene(180, 250, 0.0));
        QCOMPARE(d0.size(), 1);
        auto st = tracker.update(d0.first(), th, 0);
        QCOMPARE(st.posture, TargetStatus::PostureState::Normal);

        // 2. 倾斜 20°
        const auto dTilt = detector.detect(makeScene(180, 250, 20.0));
        QCOMPARE(dTilt.size(), 1);
        st = tracker.update(dTilt.first(), th, 300);
        QCOMPARE(st.posture, TargetStatus::PostureState::Tilted);

        // 3. 完全倾倒 90° 横躺在桌面
        const auto dFall = detector.detect(makeScene(190, 275, 90.0));
        QCOMPARE(dFall.size(), 1);
        st = tracker.update(dFall.first(), th, 600);
        QVERIFY2(st.posture == TargetStatus::PostureState::Toppled,
                 qPrintable(QString("expected Toppled got deltaAngle=%1 w=%2 h=%3")
                                .arg(st.deltaAngle).arg(dFall.first().box.width).arg(dFall.first().box.height)));
        QVERIFY(st.hasEverToppled);

        // 4. 倾倒后继续横向移动
        const auto dFallMove = detector.detect(makeScene(290, 275, 90.0));
        QCOMPARE(dFallMove.size(), 1);
        st = tracker.update(dFallMove.first(), th, 900);
        QCOMPARE(st.posture, TargetStatus::PostureState::Toppled);
        QCOMPARE(st.motion, TargetStatus::MotionState::Displaced);
        QVERIFY(st.hasEverMoved);

        // 5. 扶正放回原位 -> 姿态恢复正常，但保留“曾移动过”
        const auto dRestored = detector.detect(makeScene(180, 250, 0.0));
        QCOMPARE(dRestored.size(), 1);
        st = tracker.update(dRestored.first(), th, 1200);
        QCOMPARE(st.posture, TargetStatus::PostureState::Normal);
        QCOMPARE(st.motion, TargetStatus::MotionState::MovedReturned);

        // 6. 非彩色灰阶立柱倾倒 90° 也能可靠跟住并报倾倒
        auto makeGrayScene = [](bool fallen) {
            cv::Mat img(480, 640, CV_8UC3, cv::Scalar(48, 50, 52));
            if (!fallen) {
                cv::rectangle(img, cv::Rect(170, 170, 56, 140), cv::Scalar(205, 205, 205), -1);
                cv::rectangle(img, cv::Rect(184, 195, 28, 85), cv::Scalar(120, 120, 120), -1);
            } else {
                cv::rectangle(img, cv::Rect(130, 212, 140, 56), cv::Scalar(205, 205, 205), -1);
                cv::rectangle(img, cv::Rect(155, 226, 85, 28), cv::Scalar(120, 120, 120), -1);
            }
            return img;
        };
        AutoContourDetector grayDet;
        TargetStateTracker grayTracker(0);
        grayDet.selectRoi(cv::Rect(162, 162, 72, 156));
        const auto g0 = grayDet.detect(makeGrayScene(false));
        QCOMPARE(g0.size(), 1);
        grayTracker.update(g0.first(), th, 0);
        const auto gFall = grayDet.detect(makeGrayScene(true));
        QCOMPARE(gFall.size(), 1);
        const auto gSt = grayTracker.update(gFall.first(), th, 300);
        QCOMPARE(gSt.posture, TargetStatus::PostureState::Toppled);
    }
    void uiDisplaysLiveTargetStateWithoutCalibration() {
        QTemporaryDir directory;
        MainWindow window(nullptr, directory.filePath("camera.ini"));
        prepare(window);
        const auto channels = window.findChildren<Channel *>();

        auto makeTrackedFrame = [&](Channel *ch, const TargetStatus &ts) {
            auto r = normal(ch);
            r.baselineCalibrated = false;
            r.trackingActive = true;
            r.anomaly.type = AnomalyType::Uncalibrated;
            r.targetStatus = ts;
            return r;
        };

        auto *motionLbl = window.findChild<QLabel *>("state-motion");
        auto *sizeLbl = window.findChild<QLabel *>("state-size");
        auto *postureLbl = window.findChild<QLabel *>("state-posture");
        auto *banner = window.findChild<QLabel *>("fusion-status");
        QVERIFY(motionLbl && sizeLbl && postureLbl && banner);

        // 1. 刚框选锁定（未手动标定基准）：立即输出 静止 / 正常 / 未倾倒
        TargetStatus stStill;
        stStill.active = true;
        stStill.visible = true;
        stStill.motion = TargetStatus::MotionState::Stationary;
        stStill.size = TargetStatus::SizeState::Normal;
        stStill.posture = TargetStatus::PostureState::Normal;
        stStill.motionText = QStringLiteral("未移动");
        stStill.sizeText = QStringLiteral("尺寸正常(100%)");
        stStill.postureText = QStringLiteral("未倾倒");
        stStill.summaryText = QStringLiteral("未移动 · 尺寸正常(100%) · 未倾倒");
        stStill.shortTag = QStringLiteral("未移动 · 正常");
        for (auto *ch : channels) {
            ch->stateChanged(MjpegStream::State::Streaming, QStringLiteral("test"));
            ch->resultReady(makeTrackedFrame(ch, stStill));
        }
        QCOMPARE(motionLbl->property("monitorValue").toString(), QStringLiteral("stationary"));
        QCOMPARE(sizeLbl->property("monitorValue").toString(), QStringLiteral("normal"));
        QCOMPARE(postureLbl->property("monitorValue").toString(), QStringLiteral("normal"));
        QCOMPARE(banner->property("monitorState").toString(), QStringLiteral("uncalibrated"));
        QVERIFY(banner->text().contains(QStringLiteral("未移动")));

        // 2. 目标已移动 + 变大
        TargetStatus stMoveGrow = stStill;
        stMoveGrow.motion = TargetStatus::MotionState::Displaced;
        stMoveGrow.currentlyDisplaced = true;
        stMoveGrow.hasEverMoved = true;
        stMoveGrow.displacement = 38.0;
        stMoveGrow.maxDisplacement = 38.0;
        stMoveGrow.size = TargetStatus::SizeState::Larger;
        stMoveGrow.scaleRatio = 1.28;
        stMoveGrow.sizeChangePct = 28.0;
        stMoveGrow.summaryText = QStringLiteral("已移动 Δ38px · 变大 +28% · 未倾倒");
        stMoveGrow.shortTag = QStringLiteral("变大 +28% · 已移动 Δ38px");
        channels[0]->resultReady(makeTrackedFrame(channels[0], stMoveGrow));
        QCOMPARE(motionLbl->property("monitorValue").toString(), QStringLiteral("displaced"));
        QCOMPARE(sizeLbl->property("monitorValue").toString(), QStringLiteral("larger"));
        QVERIFY(sizeLbl->text().contains(QStringLiteral("变大")));

        // 3. 移回原位但已倾倒 -> 同时显示“曾移动过(已回位)”和“已倾倒”
        TargetStatus stReturnedToppled = stStill;
        stReturnedToppled.motion = TargetStatus::MotionState::MovedReturned;
        stReturnedToppled.hasEverMoved = true;
        stReturnedToppled.maxDisplacement = 38.0;
        stReturnedToppled.posture = TargetStatus::PostureState::Toppled;
        stReturnedToppled.hasEverToppled = true;
        stReturnedToppled.angleReliable = true;
        stReturnedToppled.deltaAngle = 82.0;
        stReturnedToppled.summaryText = QStringLiteral("曾移动过(最大38px,已回位) · 尺寸正常(100%) · 已倾倒(82°)");
        stReturnedToppled.shortTag = QStringLiteral("已倾倒 82° · 曾移动过(已回位)");
        channels[0]->resultReady(makeTrackedFrame(channels[0], stReturnedToppled));
        QCOMPARE(motionLbl->property("monitorValue").toString(), QStringLiteral("moved_returned"));
        QCOMPARE(postureLbl->property("monitorValue").toString(), QStringLiteral("toppled"));
        QVERIFY(motionLbl->text().contains(QStringLiteral("曾移动过")));
        QVERIFY(postureLbl->text().contains(QStringLiteral("已倾倒")));
    }
    void antiInterferenceRejectsTinyYellowStickerAndRequiresConfirmation() {
        auto drawYellowBottle = [](cv::Mat &img, int cx, int cy) {
            cv::rectangle(img, cv::Rect(cx - 28, cy - 60, 56, 130), cv::Scalar(25, 205, 235), -1);
            cv::rectangle(img, cv::Rect(cx - 14, cy - 82, 28, 24), cv::Scalar(25, 205, 235), -1);
            cv::rectangle(img, cv::Rect(cx - 15, cy - 92, 30, 12), cv::Scalar(20, 185, 220), -1);
            cv::rectangle(img, cv::Rect(cx - 26, cy - 12, 52, 34), cv::Scalar(225, 236, 225), -1);
            cv::line(img, cv::Point(cx - 16, cy + 4), cv::Point(cx + 16, cy + 4), cv::Scalar(40, 110, 40), 3);
        };
        cv::Mat cam2(480, 640, CV_8UC3, cv::Scalar(125, 128, 130));
        drawYellowBottle(cam2, 320, 260);

        AutoContourDetector srcDet;
        srcDet.setYoloEnabled(false); // 专门验证纯视觉多特征下也能拒绝对角微小黄色干扰贴纸
        srcDet.selectRoi(cv::Rect(286, 162, 68, 174));
        const auto dSrc = srcDet.detect(cam2);
        QCOMPARE(dSrc.size(), 1);
        const TargetSignature sig = srcDet.exportSignature(cam2);
        QVERIFY(sig.valid);

        // 模拟检测点 1：灰白金属柜 + 画面最左侧边缘一个 26x26 黄色小标签（无瓶子）
        cv::Mat cam1Cabinet(480, 640, CV_8UC3, cv::Scalar(185, 188, 190));
        cv::rectangle(cam1Cabinet, cv::Rect(20, 110, 220, 290), cv::Scalar(172, 176, 178), 2);
        cv::rectangle(cam1Cabinet, cv::Rect(260, 110, 320, 290), cv::Scalar(168, 172, 175), 2);
        cv::rectangle(cam1Cabinet, cv::Rect(4, 240, 26, 26), cv::Scalar(20, 190, 235), -1);

        AutoContourDetector peerDet;
        peerDet.setYoloEnabled(false);
        peerDet.setCrossViewConfirmFrames(3);
        peerDet.adoptSignature(sig);
        for (int i = 0; i < 6; ++i) {
            const auto res = peerDet.detect(cam1Cabinet);
            QVERIFY2(res.isEmpty(), "Must not lock onto tiny yellow sticker on cabinet!");
        }

        // 模拟 1~2 帧瞬时闪现干扰：未达 3 帧连续确认前绝不能抢锁
        cv::Mat cam1WithBottle = cam1Cabinet.clone();
        drawYellowBottle(cam1WithBottle, 380, 260);
        QVERIFY(peerDet.detect(cam1WithBottle).isEmpty()); // 第 1 帧候选
        QVERIFY(peerDet.detect(cam1WithBottle).isEmpty()); // 第 2 帧候选
        QVERIFY(peerDet.detect(cam1Cabinet).isEmpty());    // 中断复位

        // 真正目标连续出现 3 帧 -> 稳定锁定
        QVERIFY(peerDet.detect(cam1WithBottle).isEmpty());
        QVERIFY(peerDet.detect(cam1WithBottle).isEmpty());
        const auto locked = peerDet.detect(cam1WithBottle);
        QCOMPARE(locked.size(), 1);
        QVERIFY(std::abs(locked.first().center.x - 380.0) < 18.0);
    }
    void boxSmoothingPreventsFalseShrinkageUnderGlare() {
        auto drawGlareBottle = [](cv::Mat &img, int cx, int cy, int glareW, int glareH) {
            cv::rectangle(img, cv::Rect(cx - 28, cy - 60, 56, 130), cv::Scalar(25, 205, 235), -1);
            cv::rectangle(img, cv::Rect(cx - 14, cy - 82, 28, 24), cv::Scalar(25, 205, 235), -1);
            cv::rectangle(img, cv::Rect(cx - 15, cy - 92, 30, 12), cv::Scalar(20, 185, 220), -1);
            cv::rectangle(img, cv::Rect(cx - 26, cy - 12, 52, 34), cv::Scalar(230, 238, 232), -1);
            if (glareW > 0 && glareH > 0) {
                // 模拟瓶身强反光/阴影洗白局部色块（瓶子本身尺寸未变）
                cv::rectangle(img, cv::Rect(cx - glareW / 2, cy - 58, glareW, glareH),
                              cv::Scalar(235, 238, 240), -1);
            }
        };

        cv::Mat base(480, 640, CV_8UC3, cv::Scalar(125, 128, 130));
        drawGlareBottle(base, 300, 250, 0, 0);

        AutoContourDetector det;
        det.setYoloEnabled(false);
        det.setBoxSmoothingEnabled(true);
        det.selectRoi(cv::Rect(268, 154, 64, 174));
        const auto d0 = det.detect(base);
        QCOMPARE(d0.size(), 1);

        TargetStateTracker tracker(0);
        const MonitorThresholds th;
        tracker.update(d0.first(), th, 0);

        for (int i = 1; i <= 10; ++i) {
            cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(125, 128, 130));
            // 交替加入瓶身上半截反光，测试锁定框不发生“局部截断缩框棘轮”
            drawGlareBottle(frame, 300, 250, (i % 2 == 0) ? 46 : 0, (i % 2 == 0) ? 44 : 0);
            const auto tracked = det.detect(frame);
            QCOMPARE(tracked.size(), 1);
            const auto st = tracker.update(tracked.first(), th, i * 100);
            QVERIFY2(st.scaleRatio >= 0.86 && st.scaleRatio <= 1.15,
                     qPrintable(QString("frame %1 unexpected scaleRatio=%2 (box %3x%4)")
                                    .arg(i).arg(st.scaleRatio)
                                    .arg(tracked.first().box.width).arg(tracked.first().box.height)));
            QCOMPARE(st.size, TargetStatus::SizeState::Normal);
        }
    }
    void yoloDetectorLoadsAndRejectsCabinetInterference() {
        auto &yolo = YoloDetector::instance();
        if (!yolo.ensureLoaded()) {
            QSKIP("YOLO26n model or ONNX Runtime not available in this environment");
        }
        QVERIFY(yolo.isReady());
        QCOMPARE(YoloDetector::chineseClassName(39), QStringLiteral("瓶子"));

        // 模拟金属柜 + 黄色小标签背景：正立与 90° 旋转都不应误检出任何瓶子
        cv::Mat cabinet(480, 640, CV_8UC3, cv::Scalar(185, 188, 190));
        cv::rectangle(cabinet, cv::Rect(20, 110, 220, 290), cv::Scalar(170, 174, 176), 2);
        cv::rectangle(cabinet, cv::Rect(4, 240, 26, 26), cv::Scalar(20, 190, 235), -1);
        const auto dets = yolo.detect(cabinet, 39, true);
        int bottleCount = 0;
        for (const auto &b : dets) {
            if (b.classId == 39) ++bottleCount;
        }
        QCOMPARE(bottleCount, 0);
    }
    void stationaryBoxJitterDoesNotReportMoving() {
        // 1. 测试 TargetStateTracker：目标已移位 95px 后静止，即使高频框抖动产生 30px/s 瞬时速度但净位移仅 3px，也绝不跳变为“正在移动”
        auto makeDet = [](double cx, double cy) {
            Detection d;
            d.valid = true;
            d.center = cv::Point2d(cx, cy);
            d.box = cv::Rect(int(std::lround(cx - 35)), int(std::lround(cy - 95)), 70, 190);
            d.area = 70.0 * 190.0;
            d.aspectRatio = 190.0 / 70.0;
            d.angle = 90.0;
            d.angleReliable = true;
            return d;
        };
        TargetStateTracker tracker(400);
        const MonitorThresholds th;
        tracker.update(makeDet(200, 250), th, 0, 0.0, 0.0);
        // 先真实移位 95px 并稳定停住
        tracker.update(makeDet(295, 250), th, 100, 0.0, 0.0);
        auto st = tracker.update(makeDet(295, 250), th, 550, 0.0, 0.0);
        QCOMPARE(st.motion, TargetStatus::MotionState::Displaced);
        QVERIFY(!st.currentlyMoving);

        // 模拟每 50ms 来回微抖 ±3px（高瞬时速度但窗口净位移很小），绝不误报“正在移动”
        for (int i = 1; i <= 12; ++i) {
            const double jitterX = (i % 2 == 0) ? 298.0 : 293.0;
            st = tracker.update(makeDet(jitterX, 250), th, 550 + i * 50, 32.0, 3.5);
            QCOMPARE(st.motion, TargetStatus::MotionState::Displaced);
            QVERIFY(!st.currentlyMoving);
        }

        // 真实持续移动（窗口净位移 >= 8px 且速度 >= 24px/s 持续 >= 220ms）才切入“正在移动”
        tracker.update(makeDet(315, 250), th, 1300, 48.0, 16.0);
        st = tracker.update(makeDet(330, 250), th, 1550, 52.0, 22.0);
        QCOMPARE(st.motion, TargetStatus::MotionState::Moving);
        QVERIFY(st.currentlyMoving);

        // 2. 测试 AutoContourDetector 的位置死区：静止目标加入 ±2px 画面微扰时，输出框中心锁死不乱晃
        auto drawBottle = [](int cx, int cy) {
            cv::Mat img(480, 640, CV_8UC3, cv::Scalar(125, 128, 130));
            cv::rectangle(img, cv::Rect(cx - 28, cy - 80, 56, 160), cv::Scalar(25, 205, 235), -1);
            cv::rectangle(img, cv::Rect(cx - 24, cy - 16, 48, 32), cv::Scalar(230, 238, 232), -1);
            return img;
        };
        AutoContourDetector det;
        det.setYoloEnabled(false);
        det.setBoxSmoothingEnabled(true);
        det.selectRoi(cv::Rect(288, 166, 64, 168));
        const auto d0 = det.detect(drawBottle(320, 250));
        QCOMPARE(d0.size(), 1);
        const cv::Point2d c0 = d0.first().center;
        for (int i = 0; i < 8; ++i) {
            const auto d = det.detect(drawBottle(320 + ((i % 2 == 0) ? 2 : -2), 250));
            QCOMPARE(d.size(), 1);
            QVERIFY2(cv::norm(d.first().center - c0) <= 2.5,
                     qPrintable(QString("expected frozen center near (%1,%2) got (%3,%4)")
                                    .arg(c0.x).arg(c0.y).arg(d.first().center.x).arg(d.first().center.y)));
        }
    }
};

QTEST_MAIN(MonitorTest)
#include "monitor_test.moc"
