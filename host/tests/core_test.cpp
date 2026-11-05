#include <QSignalSpy>
#include <QtTest>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "colorblobdetector.h"
#include "devicediscovery.h"
#include "mjpegparser.h"
#include "mjpegstream.h"

namespace {
QByteArray encodeJpeg(const cv::Scalar &color, int w = 64, int h = 48)
{
    cv::Mat img(h, w, CV_8UC3, color);
    std::vector<uchar> buf;
    cv::imencode(".jpg", img, buf);
    return QByteArray(reinterpret_cast<const char *>(buf.data()), int(buf.size()));
}

QByteArray part(const QByteArray &jpeg, bool withLength = true)
{
    QByteArray out = "\r\n--frame\r\nContent-Type: image/jpeg\r\n";
    if (withLength)
        out += "Content-Length: " + QByteArray::number(jpeg.size()) + "\r\n";
    return out + "\r\n" + jpeg;
}

// 640x480 灰色背景上画一个指定颜色的实心圆
cv::Mat sceneWithCircle(const cv::Point &center, int radius, const cv::Scalar &bgr)
{
    cv::Mat img(480, 640, CV_8UC3, cv::Scalar(90, 90, 90));
    cv::circle(img, center, radius, bgr, cv::FILLED);
    return img;
}
} // namespace

class CoreTest : public QObject
{
    Q_OBJECT
private slots:
    void parserSplitsFramesWithContentLength();
    void parserHandlesByteByByteInput();
    void parserFallsBackToMarkers();
    void parserSkipsInvalidLength();
    void parserIgnoresHeaderLikeBytesInJpeg();
    void detectorFindsBlueCircle();
    void detectorFindsRedAcrossHueZero();
    void detectorIgnoresSmallNoise();
    void detectorLimitsTargets();
    void pickColorProducesWorkingRange();
    void pickColorWrapsRed();
    void discoveryParsesAnnounce();
    void discoveryRejectsBadAnnounce();
    void streamReceivesFromFakeCamera();
};

void CoreTest::parserSplitsFramesWithContentLength()
{
    const QByteArray a = encodeJpeg(cv::Scalar(255, 0, 0));
    const QByteArray b = encodeJpeg(cv::Scalar(0, 255, 0));
    MjpegParser parser;
    const auto frames = parser.feed(part(a) + part(b));
    QCOMPARE(frames.size(), 2);
    QCOMPARE(frames[0], a);
    QCOMPARE(frames[1], b);
    QCOMPARE(parser.droppedCount(), 0);
}

void CoreTest::parserHandlesByteByByteInput()
{
    const QByteArray a = encodeJpeg(cv::Scalar(10, 20, 30));
    const QByteArray stream = part(a) + part(a) + part(a);
    MjpegParser parser;
    int count = 0;
    for (char c : stream)
        for (const QByteArray &f : parser.feed(QByteArray(1, c))) {
            QCOMPARE(f, a);
            ++count;
        }
    QCOMPARE(count, 3);
}

void CoreTest::parserFallsBackToMarkers()
{
    const QByteArray a = encodeJpeg(cv::Scalar(1, 2, 3));
    MjpegParser parser;
    auto frames = parser.feed(part(a, false));
    QCOMPARE(frames.size(), 1);
    QCOMPARE(frames[0], a);
    // 完全没有分段头的裸 JPEG 序列
    frames = parser.feed(a + a);
    QCOMPARE(frames.size(), 2);
}

void CoreTest::parserSkipsInvalidLength()
{
    const QByteArray a = encodeJpeg(cv::Scalar(5, 5, 5));
    QByteArray bad = "\r\n--frame\r\nContent-Length: 99999999\r\n\r\n";
    MjpegParser parser;
    const auto frames = parser.feed(bad + part(a));
    QCOMPARE(frames.size(), 1);
    QCOMPARE(frames[0], a);
    QVERIFY(parser.droppedCount() >= 1);
}

void CoreTest::parserIgnoresHeaderLikeBytesInJpeg()
{
    // 构造一个内部含有 "\r\n\r\n" 的假 JPEG，长度正确时应原样取出
    QByteArray fake("\xFF\xD8", 2);
    fake += QByteArray(2000, 'x') + "\r\n\r\nContent-Length: 5\r\n\r\n" + QByteArray(100, 'y');
    fake += QByteArray("\xFF\xD9", 2);
    MjpegParser parser;
    const auto frames = parser.feed(part(fake) + part(fake));
    QCOMPARE(frames.size(), 2);
    QCOMPARE(frames[0], fake);
}

void CoreTest::detectorFindsBlueCircle()
{
    ColorBlobDetector detector;  // 默认参数为蓝色
    const auto result = detector.detect(sceneWithCircle({400, 150}, 30, cv::Scalar(255, 0, 0)));
    QCOMPARE(result.size(), 1);
    QVERIFY(std::abs(result[0].center.x - 400) < 1.0);
    QVERIFY(std::abs(result[0].center.y - 150) < 1.0);
    QVERIFY(std::abs(result[0].area - 3.14159 * 30 * 30) < 150);
}

void CoreTest::detectorFindsRedAcrossHueZero()
{
    ColorBlobDetector detector;
    ColorParams p;
    p.hMin = 170;
    p.hMax = 10;
    detector.setParams(p);
    // 一个 H≈0 的红圆和一个 H≈175 的红圆，应该都能检出
    cv::Mat img = sceneWithCircle({150, 240}, 40, cv::Scalar(0, 0, 255));
    cv::circle(img, {480, 240}, 30, cv::Scalar(60, 0, 230), cv::FILLED);
    p.maxTargets = 5;
    detector.setParams(p);
    const auto result = detector.detect(img);
    QCOMPARE(result.size(), 2);
    QVERIFY(std::abs(result[0].center.x - 150) < 1.0);  // 面积大的排在前面
    QVERIFY(std::abs(result[1].center.x - 480) < 1.0);
}

void CoreTest::detectorIgnoresSmallNoise()
{
    ColorBlobDetector detector;
    cv::Mat img = sceneWithCircle({320, 240}, 25, cv::Scalar(255, 0, 0));
    for (int i = 0; i < 20; ++i)
        cv::circle(img, {30 + i * 30, 450}, 1, cv::Scalar(255, 0, 0), cv::FILLED);
    const auto result = detector.detect(img);
    QCOMPARE(result.size(), 1);
    QVERIFY(std::abs(result[0].center.x - 320) < 1.0);

    // 画面中没有目标颜色
    QVERIFY(detector.detect(sceneWithCircle({320, 240}, 25, cv::Scalar(0, 255, 0))).isEmpty());
}

void CoreTest::detectorLimitsTargets()
{
    ColorBlobDetector detector;
    cv::Mat img = sceneWithCircle({100, 100}, 20, cv::Scalar(255, 0, 0));
    cv::circle(img, {300, 300}, 35, cv::Scalar(255, 0, 0), cv::FILLED);
    cv::circle(img, {500, 100}, 25, cv::Scalar(255, 0, 0), cv::FILLED);
    auto result = detector.detect(img);
    QCOMPARE(result.size(), 1);  // 默认只返回最大的
    QVERIFY(std::abs(result[0].center.x - 300) < 1.0);

    ColorParams p = detector.params();
    p.maxTargets = 3;
    detector.setParams(p);
    result = detector.detect(img);
    QCOMPARE(result.size(), 3);
    QVERIFY(result[0].area > result[1].area && result[1].area > result[2].area);
}

void CoreTest::pickColorProducesWorkingRange()
{
    const cv::Mat img = sceneWithCircle({200, 200}, 40, cv::Scalar(0, 200, 0));  // 绿色
    ColorParams p;
    QVERIFY(ColorBlobDetector::pickColor(img, 200, 200, &p));
    QVERIFY(p.hMin < 60 && p.hMax > 60);
    ColorBlobDetector detector;
    detector.setParams(p);
    const auto result = detector.detect(img);
    QCOMPARE(result.size(), 1);
    QVERIFY(std::abs(result[0].center.x - 200) < 1.0);

    QVERIFY(!ColorBlobDetector::pickColor(img, -1, 10, &p));
    QVERIFY(!ColorBlobDetector::pickColor(img, 640, 10, &p));
}

void CoreTest::pickColorWrapsRed()
{
    const cv::Mat img = sceneWithCircle({300, 200}, 40, cv::Scalar(0, 0, 255));
    ColorParams p;
    QVERIFY(ColorBlobDetector::pickColor(img, 300, 200, &p));
    QVERIFY2(p.hMin > p.hMax, "红色的范围应跨过 0");
    ColorBlobDetector detector;
    detector.setParams(p);
    QCOMPARE(detector.detect(img).size(), 1);
}

void CoreTest::discoveryParsesAnnounce()
{
    const QByteArray msg = R"({"type":"announce","version":1,"nonce":"abc","id":2,)"
                           R"("ip":"10.0.0.9","stream":"http://10.0.0.9:81/stream","control_port":5005})";
    DeviceDiscovery::Device d;
    QVERIFY(DeviceDiscovery::parseAnnounce(msg, "abc", QHostAddress("192.168.4.7"), &d));
    QCOMPARE(d.id, 2);
    QCOMPARE(d.controlPort, quint16(5005));
    // 视频地址使用实际来源 IP，而不是设备自报的 IP
    QCOMPARE(d.streamUrl, QUrl("http://192.168.4.7:81/stream"));

    // IPv4 映射的 IPv6 地址
    QVERIFY(DeviceDiscovery::parseAnnounce(msg, "abc", QHostAddress("::ffff:192.168.4.7"), &d));
    QCOMPARE(d.address, QHostAddress("192.168.4.7"));
}

void CoreTest::discoveryRejectsBadAnnounce()
{
    const QHostAddress from("192.168.1.5");
    DeviceDiscovery::Device d;
    const auto make = [](const QByteArray &nonce, int id, const QByteArray &stream) {
        return R"({"type":"announce","version":1,"nonce":")" + nonce + R"(","id":)"
               + QByteArray::number(id) + R"(,"stream":")" + stream + R"("})";
    };
    QVERIFY(DeviceDiscovery::parseAnnounce(make("n", 1, "http://x:81/stream"), "n", from, &d));
    QVERIFY(!DeviceDiscovery::parseAnnounce(make("other", 1, "http://x:81/stream"), "n", from, &d));
    QVERIFY(!DeviceDiscovery::parseAnnounce(make("n", 0, "http://x:81/stream"), "n", from, &d));
    QVERIFY(!DeviceDiscovery::parseAnnounce(make("n", 4, "http://x:81/stream"), "n", from, &d));
    QVERIFY(!DeviceDiscovery::parseAnnounce(make("n", 1, "ftp://x:81/stream"), "n", from, &d));
    QVERIFY(!DeviceDiscovery::parseAnnounce(make("n", 1, "http://x/stream"), "n", from, &d));
    QVERIFY(!DeviceDiscovery::parseAnnounce("not json", "n", from, &d));
}

void CoreTest::streamReceivesFromFakeCamera()
{
    // 需要先运行 tools/fake_esp32cam.py，并设置 FAKE_CAM_URL=http://127.0.0.1:8081/stream
    const QString url = qEnvironmentVariable("FAKE_CAM_URL");
    if (url.isEmpty())
        QSKIP("未设置 FAKE_CAM_URL，跳过联机测试");

    MjpegStream stream;
    QSignalSpy frames(&stream, &MjpegStream::frameDecoded);
    stream.open(QUrl(url));
    QTRY_VERIFY_WITH_TIMEOUT(frames.count() >= 20, 10000);
    const cv::Mat first = frames.first().at(0).value<cv::Mat>();
    QCOMPARE(first.cols, 640);
    QCOMPARE(first.rows, 480);
    stream.close();
    QCOMPARE(stream.state(), MjpegStream::State::Idle);
}

QTEST_MAIN(CoreTest)
#include "core_test.moc"
