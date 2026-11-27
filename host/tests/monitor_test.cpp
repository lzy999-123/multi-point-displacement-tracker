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
};

QTEST_MAIN(MonitorTest)
#include "monitor_test.moc"
