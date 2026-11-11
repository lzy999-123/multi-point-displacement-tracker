#include <QPainter>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QFile>
#include <QtTest>
#include "videowidget.h"
#include "cameraconfigpath.h"

namespace {
ChannelResult scene()
{
    ChannelResult result;
    result.image = QImage(400, 300, QImage::Format_RGB888);
    result.image.fill(Qt::black);
    QPainter painter(&result.image);
    painter.fillRect(QRect(50, 125, 50, 50), Qt::red);
    painter.fillRect(QRect(300, 125, 50, 50), Qt::blue);
    return result;
}

QImage render(VideoWidget &widget)
{
    QImage output(widget.size(), QImage::Format_ARGB32);
    output.fill(Qt::transparent);
    widget.render(&output);
    return output;
}
}

class ViewerTest : public QObject
{
    Q_OBJECT
private slots:
    void rotationPreservesSourceAndMapsClicks();
    void detectionAndMaskRotateWithPicture();
    void letterboxClicksStayOutOfSource();
    void creatorAndDemoIgnoreStaleAdjacentConfigs();
    void standaloneUsesAdjacentConfig();
    void projectWithoutSettingsKeepsStandaloneConfig();
};

void ViewerTest::rotationPreservesSourceAndMapsClicks()
{
    auto result = scene();
    const QImage source = result.image.copy();
    VideoWidget widget("Camera 1");
    widget.resize(800, 600);
    widget.setResult(result);
    QSignalSpy clicks(&widget, &VideoWidget::imageClicked);
    const QImage plain = render(widget);
    QCOMPARE(plain.pixelColor(150, 300), QColor(Qt::red));
    QTest::mouseClick(&widget, Qt::LeftButton, Qt::NoModifier, QPoint(150, 300));
    QCOMPARE(clicks.takeFirst(), QVariantList({75, 150}));

    widget.setRotate180(true);
    const QImage rotated = render(widget);
    QCOMPARE(rotated.pixelColor(150, 300), QColor(Qt::blue));
    QCOMPARE(rotated.pixelColor(650, 300), QColor(Qt::red));
    QTest::mouseClick(&widget, Qt::LeftButton, Qt::NoModifier, QPoint(150, 300));
    QCOMPARE(clicks.takeFirst(), QVariantList({324, 149}));
    QCOMPARE(result.image, source);
    QCOMPARE(plain.copy(0, 0, 240, 90), rotated.copy(0, 0, 240, 90));

    widget.setRotate180(false);
    QCOMPARE(render(widget).pixelColor(150, 300), QColor(Qt::red));
}

void ViewerTest::detectionAndMaskRotateWithPicture()
{
    auto result = scene();
    Detection detection;
    detection.box = cv::Rect(50, 125, 50, 50);
    detection.center = cv::Point2d(75, 150);
    result.detections.append(detection);
    VideoWidget widget("Camera 2");
    widget.resize(400, 300);
    widget.setResult(result);
    QCOMPARE(render(widget).pixelColor(50, 160), QColor(0, 230, 0));
    widget.setRotate180(true);
    const QImage rotated = render(widget);
    QCOMPARE(rotated.pixelColor(350, 140), QColor(0, 230, 0));
    QCOMPARE(rotated.pixelColor(75, 150), QColor(Qt::blue));

    result.detections.clear();
    result.mask = QImage(400, 300, QImage::Format_Grayscale8);
    result.mask.fill(20);
    QPainter maskPainter(&result.mask);
    maskPainter.fillRect(QRect(200, 0, 200, 300), QColor(200, 200, 200));
    maskPainter.end();
    widget.setResult(result);
    widget.setShowMask(true);
    QCOMPARE(render(widget).pixelColor(75, 150), QColor(200, 200, 200));
    QCOMPARE(render(widget).pixelColor(325, 150), QColor(20, 20, 20));
}

void ViewerTest::letterboxClicksStayOutOfSource()
{
    VideoWidget widget("Camera 1");
    widget.resize(800, 800);
    widget.setResult(scene());
    widget.setRotate180(true);
    QSignalSpy clicks(&widget, &VideoWidget::imageClicked);
    QTest::mouseClick(&widget, Qt::LeftButton, Qt::NoModifier, QPoint(400, 50));
    QTest::mouseClick(&widget, Qt::LeftButton, Qt::NoModifier, QPoint(400, 750));
    QCOMPARE(clicks.size(), 0);
    QTest::mouseClick(&widget, Qt::LeftButton, Qt::NoModifier, QPoint(150, 400));
    QCOMPARE(clicks.takeFirst(), QVariantList({324, 149}));
}

void ViewerTest::creatorAndDemoIgnoreStaleAdjacentConfigs()
{
    QTemporaryDir temporary;
    QVERIFY(temporary.isValid());
    QDir root(temporary.path());
    QVERIFY(root.mkpath("host/build/Desktop_Qt_Release/release"));
    QVERIFY(root.mkpath("host/build/run"));
    const QString host = root.filePath("host");
    QFile project(host + "/DisplacementTracker.pro");
    QVERIFY(project.open(QIODevice::WriteOnly)); project.close();
    QFile canonical(host + "/camera-runtime.ini");
    QVERIFY(canonical.open(QIODevice::WriteOnly));
    canonical.write("[camera]\nrotate180=true\n"); canonical.close();
    for (const QString &relative : {QString("host/build/run"), QString("host/build/Desktop_Qt_Release/release")}) {
        const QString directory = root.filePath(relative);
        QFile stale(directory + "/camera-runtime.ini");
        QVERIFY(stale.open(QIODevice::WriteOnly));
        stale.write("[camera]\nurl=udp://192.168.110.2:5006\n"); stale.close();
        QCOMPARE(cameraSettingsPath(directory), host + "/camera-runtime.ini");
    }
}

void ViewerTest::standaloneUsesAdjacentConfig()
{
    QTemporaryDir temporary;
    QVERIFY(temporary.isValid());
    QCOMPARE(cameraSettingsPath(temporary.path()), temporary.path() + "/camera-runtime.ini");
}

void ViewerTest::projectWithoutSettingsKeepsStandaloneConfig()
{
    QTemporaryDir temporary;
    QVERIFY(temporary.isValid());
    QDir root(temporary.path());
    QVERIFY(root.mkpath("build/release"));
    QFile project(root.filePath("DisplacementTracker.pro"));
    QVERIFY(project.open(QIODevice::WriteOnly)); project.close();
    const QString executableDirectory = root.filePath("build/release");
    QCOMPARE(cameraSettingsPath(executableDirectory), executableDirectory + "/camera-runtime.ini");
}

QTEST_MAIN(ViewerTest)
#include "viewer_test.moc"
