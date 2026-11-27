#include <QSignalSpy>
#include <QNetworkDatagram>
#include <QUdpSocket>
#include <QtTest>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "colorblobdetector.h"
#include "channel.h"
#include "devicediscovery.h"
#include "mjpegparser.h"
#include "mjpegstream.h"
#include "udpstream.h"
#include "videoprofile.h"

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
    void udpReordersAndRejectsDuplicates();
    void udpRejectsWrongPeerAndMalformedChunks();
    void udpRecoversAfterCameraRestart();
    void udpWrapsAndSkipsBacklog();
    void udpAcknowledgesEveryCompleteFrame();
    void udpHandlesCollidingSequencesAndLateCompletion();
    void udpReceivesWhileFrameConsumerBlocks();
    void udpCloseDiscardsPendingAndReopens();
    void udpChannelProcessesFramesAndShutsDown();
    void udpChannelStaysOnUdpAcrossLongOutage();
    void udpRequestsOnlyMissingFreshChunks();
    void udpProfileRequiresMatchingConfirmationAndImageDimensions();
    void udpHighQualityTuneTransitionsAfterProfileConfirmationAndRecovers();
    void udpEnforcesExpectedDeviceId();
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
    QCOMPARE(detector.lastMask().at<uchar>(150, 400), uchar(255));
    QCOMPARE(detector.lastMask().at<uchar>(0, 0), uchar(0));
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
    d.udpStreamPort = 5006;
    QCOMPARE(d.bestStreamUrl(), QUrl("udp://192.168.4.7:5006"));

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

namespace {
class UdpFixture {
public:
    QUdpSocket camera;
    UdpStream stream;
    QSignalSpy decoded{&stream, &UdpStream::frameDecoded};
    QHostAddress client;
    quint16 clientPort = 0;
    QByteArray jpeg = encodeJpeg(cv::Scalar(255, 0, 0), 320, 240);
    QList<QByteArray> controls;

    UdpFixture() {
        camera.bind(QHostAddress(QHostAddress::LocalHost), 0);
        QObject::connect(&camera, &QUdpSocket::readyRead, [&] {
            while (camera.hasPendingDatagrams()) {
                const auto packet = camera.receiveDatagram();
                controls.append(packet.data());
                if (packet.data().startsWith("VREQ")) {
                    client = packet.senderAddress();
                    clientPort = packet.senderPort();
                }
            }
        });
        stream.open(QUrl(QString("udp://127.0.0.1:%1").arg(camera.localPort())));
    }
    QByteArray packet(quint16 seq, int index) {
        const int count = (int(jpeg.size()) + 1399) / 1400;
        const int offset = index * 1400, size = qMin(1400, int(jpeg.size()) - offset);
        VidPacketHeader hdr{0x31444956, 1, 0, seq, quint8(index), quint8(count),
                            quint16(size), quint32(jpeg.size())};
        QByteArray bytes(reinterpret_cast<const char *>(&hdr), sizeof(hdr));
        bytes.append(jpeg.mid(offset, size));
        return bytes;
    }
    void send(quint16 seq, QUdpSocket *sender = nullptr, bool reverse = false) {
        const int count = (int(jpeg.size()) + 1399) / 1400;
        for (int j = 0; j < count; ++j) {
            const int i = reverse ? count - 1 - j : j;
            (sender ? sender : &camera)->writeDatagram(packet(seq, i), client, clientPort);
        }
    }
    bool waitAck(quint16 seq, int timeoutMs = 1000) {
        const QByteArray ack = QByteArrayLiteral("VACK")
            + QByteArray(1, char(seq & 0xff)) + QByteArray(1, char(seq >> 8));
        QElapsedTimer timer;
        timer.start();
        // Deliberately do not process Qt events: the decoder/consumer cannot run.
        while (!controls.contains(ack) && timer.elapsed() < timeoutMs) {
            if (!camera.hasPendingDatagrams()) camera.waitForReadyRead(qMax(1, timeoutMs - int(timer.elapsed())));
            while (camera.hasPendingDatagrams()) {
                const auto datagram = camera.receiveDatagram();
                controls.append(datagram.data());
                client = datagram.senderAddress();
                clientPort = datagram.senderPort();
            }
        }
        return controls.contains(ack);
    }
};
} // namespace

void CoreTest::udpReordersAndRejectsDuplicates()
{
    UdpFixture f;
    QTRY_VERIFY(f.clientPort != 0);
    f.send(10, nullptr, true);
    QTRY_COMPARE(f.decoded.size(), 1);
    f.send(10);
    f.send(9);
    QTest::qWait(50);
    QCOMPARE(f.decoded.size(), 1);
    f.send(11);
    QTRY_COMPARE(f.decoded.size(), 2);
    QCOMPARE(f.decoded.last()[0].value<cv::Mat>().cols, 320);
}

void CoreTest::udpRejectsWrongPeerAndMalformedChunks()
{
    UdpFixture f;
    QTRY_VERIFY(f.clientPort != 0);
    QUdpSocket unrelated;
    unrelated.bind(QHostAddress(QHostAddress::LocalHost), 0);
    f.send(1, &unrelated);
    QByteArray invalid = f.packet(2, 0);
    invalid[9] = char(255); // chunk_count 与 frame_size 不一致
    f.camera.writeDatagram(invalid, f.client, f.clientPort);
    invalid = f.packet(3, 0);
    invalid.chop(1);
    f.camera.writeDatagram(invalid, f.client, f.clientPort);
    // 同一序号后续包改变 frame_size，且索引超出此前槽位。
    f.camera.writeDatagram(f.packet(4, 0), f.client, f.clientPort);
    VidPacketHeader mismatch{0x31444956, 1, 0, 4, 4, 5, 1400, 7000};
    invalid = QByteArray(reinterpret_cast<const char *>(&mismatch), sizeof(mismatch)) + QByteArray(1400, 'x');
    f.camera.writeDatagram(invalid, f.client, f.clientPort);
    QTest::qWait(80);
    QCOMPARE(f.decoded.size(), 0);
    for (const auto &control : f.controls) QVERIFY(!control.startsWith("VACK"));
    f.send(5);
    QTRY_COMPARE(f.decoded.size(), 1);
}

void CoreTest::udpRecoversAfterCameraRestart()
{
    UdpFixture f;
    QTRY_VERIFY(f.clientPort != 0);
    f.send(10000);
    QTRY_COMPARE(f.decoded.size(), 1);
    f.send(0);
    QTest::qWait(80);
    QCOMPARE(f.decoded.size(), 1); // 没有断流前不能把旧包当作重启。
    QTRY_VERIFY_WITH_TIMEOUT(f.stream.state() == MjpegStream::State::Reconnecting, 2100);
    f.send(1);
    QTRY_COMPARE(f.decoded.size(), 2);
    QCOMPARE(f.stream.state(), MjpegStream::State::Streaming);
}

void CoreTest::udpWrapsAndSkipsBacklog()
{
    UdpFixture f;
    QTRY_VERIFY(f.clientPort != 0);
    f.send(65535);
    QTRY_COMPARE(f.decoded.size(), 1);
    f.send(0);
    QTRY_COMPARE(f.decoded.size(), 2);
    // 不处理事件，先把一批完整帧送入 socket；接收时只交付最新图像。
    f.send(1);
    f.send(2);
    f.send(3);
    QVERIFY(f.waitAck(3));
    QTRY_COMPARE(f.decoded.size(), 3);
    QTest::qWait(40);
    QCOMPARE(f.decoded.size(), 3);
    f.send(2);
    QTest::qWait(40);
    QCOMPARE(f.decoded.size(), 3);
    f.send(4);
    QTRY_COMPARE(f.decoded.size(), 4);
}

void CoreTest::udpAcknowledgesEveryCompleteFrame()
{
    UdpFixture f;
    QTRY_VERIFY(f.clientPort != 0);
    QVERIFY(f.controls.first().startsWith(QByteArrayLiteral("VREQA2")));
    f.send(65535);
    QVERIFY(f.waitAck(65535));
    QCOMPARE(f.decoded.size(), 0); // ACK precedes decode on the consumer thread.
    QTRY_COMPARE(f.decoded.size(), 1);
    f.controls.clear();
    // ACK means full transport assembly; corrupt JPEGs never feed the watchdog.
    const QByteArray original = f.jpeg;
    f.jpeg = QByteArrayLiteral("broken-jpeg");
    f.send(0);
    QVERIFY(f.waitAck(0));
    QTest::qWait(50);
    QCOMPARE(f.decoded.size(), 1);
    QTRY_VERIFY_WITH_TIMEOUT(f.stream.state() == MjpegStream::State::Reconnecting, 2100);
    f.jpeg = original;
    f.send(1);
    QTRY_COMPARE(f.decoded.size(), 2);

    f.controls.clear();
    for (quint16 seq = 2; seq <= 6; ++seq) f.send(seq);
    QVERIFY(f.waitAck(6));
    for (quint16 seq = 2; seq <= 6; ++seq) QVERIFY(f.waitAck(seq));
    QTRY_COMPARE(f.decoded.size(), 3); // Five ACKs, one latest image.
}

void CoreTest::udpHandlesCollidingSequencesAndLateCompletion()
{
    UdpFixture f;
    QTRY_VERIFY(f.clientPort != 0);
    const int chunks = (f.jpeg.size() + 1399) / 1400;
    QVERIFY(chunks > 1);
    f.camera.writeDatagram(f.packet(8, 0), f.client, f.clientPort);
    f.camera.writeDatagram(f.packet(16, 0), f.client, f.clientPort);
    for (int i = 1; i < chunks; ++i)
        f.camera.writeDatagram(f.packet(16, i), f.client, f.clientPort);
    for (int i = 1; i < chunks; ++i)
        f.camera.writeDatagram(f.packet(8, i), f.client, f.clientPort);
    QVERIFY(f.waitAck(8));
    QVERIFY(f.waitAck(16));
    QTRY_COMPARE(f.decoded.size(), 1);
    f.send(15);
    QVERIFY(f.waitAck(15)); // A late complete frame releases its sender window.
    QTest::qWait(40);
    QCOMPARE(f.decoded.size(), 1); // It must never replace the newer image.
    f.send(24);
    QTRY_COMPARE(f.decoded.size(), 2);
}

void CoreTest::udpReceivesWhileFrameConsumerBlocks()
{
    UdpFixture f;
    QTRY_VERIFY(f.clientPort != 0);
    bool entered = false;
    bool acknowledgedWhileBlocked = false;
    connect(&f.stream, &UdpStream::frameDecoded, &f.stream, [&] {
        if (entered) return;
        entered = true;
        // This direct slot stands in for a slow decoder/detector/display conversion.
        for (quint16 seq = 2; seq <= 40; ++seq) f.send(seq);
        acknowledgedWhileBlocked = f.waitAck(40, 1000);
    });
    f.send(1);
    QTRY_VERIFY(entered);
    QVERIFY(acknowledgedWhileBlocked);
    for (quint16 seq = 1; seq <= 40; ++seq) QVERIFY(f.waitAck(seq));
    QTRY_COMPARE(f.decoded.size(), 2);
    QTest::qWait(40);
    QCOMPARE(f.decoded.size(), 2); // No queued history of 39 images.
}

void CoreTest::udpCloseDiscardsPendingAndReopens()
{
    UdpFixture f;
    QTRY_VERIFY(f.clientPort != 0);
    f.send(100);
    QVERIFY(f.waitAck(100));
    f.stream.close();
    QTest::qWait(30);
    QCOMPARE(f.decoded.size(), 0);
    QCOMPARE(f.stream.state(), MjpegStream::State::Idle);
    f.clientPort = 0;
    f.stream.open(QUrl(QString("udp://127.0.0.1:%1").arg(f.camera.localPort())));
    QTRY_VERIFY(f.clientPort != 0);
    f.send(1);
    QTRY_COMPARE(f.decoded.size(), 1);
    QCOMPARE(f.decoded.first()[1].toULongLong(), quint64(1));
}

void CoreTest::udpChannelProcessesFramesAndShutsDown()
{
    UdpFixture f;
    QTRY_VERIFY(f.clientPort != 0);
    f.stream.close();
    f.clientPort = 0;
    Channel channel(1);
    QSignalSpy results(&channel, &Channel::resultReady);
    connect(&channel, &Channel::resultReady, &channel, [&] { channel.acknowledgeResult(); });
    channel.setShowMask(true);
    channel.open(QUrl(QString("udp://127.0.0.1:%1").arg(f.camera.localPort())));
    QTRY_VERIFY(f.clientPort != 0);
    // Odd widths exercise QImage row padding in the direct BGR -> RGB conversion.
    f.jpeg = encodeJpeg(cv::Scalar(255, 0, 0), 65, 49);
    f.send(1);
    QTRY_COMPARE(results.size(), 1);
    const ChannelResult first = results.first()[0].value<ChannelResult>();
    QCOMPARE(first.image.size(), QSize(65, 49));
    QVERIFY(first.image.pixelColor(32, 24).blue() > 240);
    QVERIFY(first.image.pixelColor(32, 24).red() < 10);
    QVERIFY(first.hasTarget);
    QCOMPARE(first.mask.pixelColor(32, 24), QColor(Qt::white));
    f.jpeg = encodeJpeg(cv::Scalar(0, 255, 0), 65, 49);
    f.send(2);
    QTRY_COMPARE(results.size(), 2);
    const ChannelResult second = results.last()[0].value<ChannelResult>();
    QVERIFY(second.image.pixelColor(32, 24).green() > 240);
    QVERIFY(!second.hasTarget);
    QVERIFY(first.image.pixelColor(32, 24).blue() > 240); // Old result owns its image.
    channel.close();
    QTRY_VERIFY(!channel.isOpen());
    // Scope exit must stop and join both the processing and receive threads.
}

void CoreTest::udpChannelStaysOnUdpAcrossLongOutage()
{
    QUdpSocket camera;
    QVERIFY(camera.bind(QHostAddress(QHostAddress::LocalHost), 5006));
    QHostAddress client;
    quint16 clientPort = 0;
    connect(&camera, &QUdpSocket::readyRead, &camera, [&] {
        while (camera.hasPendingDatagrams()) {
            const auto request = camera.receiveDatagram();
            client = request.senderAddress();
            clientPort = request.senderPort();
        }
    });
    Channel channel(1);
    QSignalSpy results(&channel, &Channel::resultReady);
    QSignalSpy transports(&channel, &Channel::activeStreamChanged);
    connect(&channel, &Channel::resultReady, &channel, [&] { channel.acknowledgeResult(); });
    const QUrl url("udp://127.0.0.1:5006");
    channel.open(url);
    QTRY_VERIFY(clientPort != 0);
    QTest::qWait(4600);
    QCOMPARE(transports.size(), 1);
    QCOMPARE(transports.first()[0].toUrl(), url);
    QVERIFY(results.isEmpty());
    // The same UDP socket must recover after the previous HTTP fallback deadline.
    const QByteArray jpeg = encodeJpeg(cv::Scalar(0, 255, 0));
    QVERIFY(jpeg.size() <= 1400);
    const VidPacketHeader header{0x31444956, 1, 0, 99, 0, 1,
                                 quint16(jpeg.size()), quint32(jpeg.size())};
    QByteArray packet(reinterpret_cast<const char *>(&header), sizeof(header));
    packet += jpeg;
    camera.writeDatagram(packet, client, clientPort);
    QTRY_COMPARE(results.size(), 1);
    QVERIFY(results.first()[0].value<ChannelResult>().image.pixelColor(32, 24).green() > 240);
    QCOMPARE(transports.size(), 1);
    channel.close();
}

void CoreTest::udpRequestsOnlyMissingFreshChunks()
{
    UdpFixture f;
    QTRY_VERIFY(f.clientPort != 0);
    const int count = (f.jpeg.size() + 1399) / 1400;
    QVERIFY(count >= 2);
    for (int i=0;i<count;++i) if (i != 1) f.camera.writeDatagram(f.packet(42, i), f.client, f.clientPort);
    const auto hasRepair = [&] {
        for (const auto &request : f.controls) if (request.startsWith("VNCK")) return true;
        return false;
    };
    QTRY_VERIFY_WITH_TIMEOUT(hasRepair(), 500);
    int requests = 0;
    for (const auto &request : f.controls) {
        if (!request.startsWith("VNCK")) continue;
        ++requests;
        QCOMPARE(request.size(), 14);
        QCOMPARE(quint8(request[4]), quint8(42));
        QCOMPARE(quint8(request[5]), quint8(0));
        QCOMPARE(quint8(request[6]), quint8(2)); // Only chunk 1 is missing.
        for (int i=7;i<14;++i) QCOMPARE(quint8(request[i]), quint8(0));
    }
    QVERIFY(requests <= 2);
    QCOMPARE(f.decoded.size(), 0);
    f.camera.writeDatagram(f.packet(42, 1), f.client, f.clientPort);
    QTRY_COMPARE(f.decoded.size(), 1);
    int completedRequests = 0;
    for (const auto &request : f.controls) if (request.startsWith("VNCK")) ++completedRequests;
    QVERIFY(completedRequests <= 2);
    QTest::qWait(150);
    int after = 0;
    for (const auto &request : f.controls) if (request.startsWith("VNCK")) ++after;
    QCOMPARE(after, completedRequests); // Completing the image stops repair requests.
}

void CoreTest::udpProfileRequiresMatchingConfirmationAndImageDimensions()
{
    UdpFixture f;
    QTRY_VERIFY(f.clientPort != 0);
    f.stream.close();
    f.controls.clear();
    f.clientPort = 0;
    QSignalSpy applied(&f.stream, &UdpStream::videoProfileApplied);
    f.stream.open(withVideoProfile(QUrl(QString("udp://127.0.0.1:%1").arg(f.camera.localPort())),
                                  VideoProfile::Qvga25));
    QTRY_VERIFY(f.clientPort != 0);
    QByteArray request;
    for (const auto &control : f.controls) if (control.startsWith("VREQA3")) request = control;
    QCOMPARE(request.size(), 16);
    QCOMPARE(quint8(request[6]), quint8(0)); // Explicit uncapped cadence.
    QCOMPARE(quint8(request[12]), quint8(VideoProfile::Qvga25));
    QByteArray ack("VCFG");
    ack.append(char(1)); ack.append(char(VideoProfile::Qvga25));
    ack.append(request.mid(13, 2));
    for (unsigned value : {320U, 240U}) {
        ack.append(char(value)); ack.append(char(value >> 8));
    }
    ack.append(char(0)); ack.append(QByteArray(3, char(0)));
    QByteArray wrongGeneration = ack;
    wrongGeneration[6] = char(quint8(wrongGeneration[6]) + 1);
    f.camera.writeDatagram(wrongGeneration, f.client, f.clientPort);
    QTest::qWait(40);
    QCOMPARE(applied.size(), 0);
    f.camera.writeDatagram(ack, f.client, f.clientPort);
    QTRY_COMPARE(applied.size(), 1);
    QCOMPARE(applied.first()[0].toInt(), int(VideoProfile::Qvga25));
    const auto sendProfileFrame = [&](quint16 sequence, int width, int height) {
        f.jpeg = encodeJpeg(cv::Scalar(0, 255, 0), width, height);
        const int count = (f.jpeg.size() + 1399) / 1400;
        for (int i = 0; i < count; ++i) {
            QByteArray packet = f.packet(sequence, i);
            packet[5] = char(VideoProfile::Qvga25 + 1);
            f.camera.writeDatagram(packet, f.client, f.clientPort);
        }
    };
    sendProfileFrame(1, 640, 480); // Stale dimensions must never reach the UI.
    QTest::qWait(100);
    QCOMPARE(f.decoded.size(), 0);
    sendProfileFrame(2, 320, 240);
    QTRY_COMPARE(f.decoded.size(), 1);
    const cv::Mat frame = f.decoded.first()[0].value<cv::Mat>();
    QCOMPARE(frame.cols, 320);
    QCOMPARE(frame.rows, 240);
}

void CoreTest::udpHighQualityTuneTransitionsAfterProfileConfirmationAndRecovers()
{
    UdpFixture f;
    f.stream.close();
    f.controls.clear();
    f.clientPort = 0;
    QSignalSpy applied(&f.stream, &UdpStream::videoProfileApplied);
    QUrl url(QString("udp://127.0.0.1:%1?transport=simple&hq=1&id=1").arg(f.camera.localPort()));
    f.stream.open(withVideoProfile(url, VideoProfile::Vga12));
    QTRY_VERIFY(f.clientPort != 0);
    QByteArray v3;
    for (const auto &c : f.controls) if (c.startsWith("VREQA3")) v3 = c;
    QCOMPARE(v3.size(), 16);
    QCOMPARE(quint8(v3[12]), quint8(VideoProfile::Vga12));
    QCOMPARE(quint8(v3[15]), quint8(3));

    f.controls.clear();
    QByteArray ack("VCFG");
    ack.append(char(1)); ack.append(char(VideoProfile::Vga12));
    ack.append(v3.mid(13, 2));
    for (unsigned value : {640U, 480U}) {
        ack.append(char(value)); ack.append(char(value >> 8));
    }
    ack.append(char(0)); ack.append(char(0)); ack.append(char(3)); ack.append(char(0));
    f.camera.writeDatagram(ack, f.client, f.clientPort);
    QTRY_COMPARE(applied.size(), 1);
    QTRY_VERIFY(!f.controls.isEmpty());
    QByteArray v2;
    for (const auto &c : f.controls) if (c.startsWith("VREQ02")) v2 = c;
    QCOMPARE(v2.size(), 12);
    const auto preset = VideoProfile::preset(VideoProfile::Vga12);
    QCOMPARE(quint8(v2[6]), VideoProfile::hqTuneFps(VideoProfile::Vga12));
    QCOMPARE(quint8(v2[7]), preset.minimumQuality);
    QCOMPARE(quint8(v2[8]), preset.maximumQuality);
    const quint16 target = quint8(v2[10]) | (quint16(quint8(v2[11])) << 8);
    QCOMPARE(target, preset.targetBytes);

    // After VREQ02 handoff, v25 cameras send reserved=0; exact resolution check still protects UI.
    f.jpeg = encodeJpeg(cv::Scalar(0, 255, 0), 320, 240);
    f.send(1);
    QTest::qWait(80);
    QCOMPARE(f.decoded.size(), 0);
    f.jpeg = encodeJpeg(cv::Scalar(0, 255, 0), 640, 480);
    f.send(2);
    QTRY_COMPARE(f.decoded.size(), 1);

    // Watchdog timeout must fall back to VREQA3 to re-lock resolution after camera reboot.
    f.controls.clear();
    QTRY_VERIFY_WITH_TIMEOUT(f.stream.state() == MjpegStream::State::Reconnecting, 2200);
    bool sawV3Again = false;
    for (const auto &c : f.controls) if (c.startsWith("VREQA3")) sawV3Again = true;
    QVERIFY(sawV3Again);
}

void CoreTest::udpEnforcesExpectedDeviceId()
{
    UdpFixture f;
    f.stream.close();
    f.controls.clear();
    f.clientPort = 0;
    f.stream.open(QUrl(QString("udp://127.0.0.1:%1?transport=simple&id=1").arg(f.camera.localPort())));
    QTRY_VERIFY(f.clientPort != 0);
    // Packet claiming device_id=2 must be dropped when channel expects id=1.
    const int count = (f.jpeg.size() + 1399) / 1400;
    for (int i = 0; i < count; ++i) {
        QByteArray p = f.packet(1, i);
        p[4] = char(2);
        f.camera.writeDatagram(p, f.client, f.clientPort);
    }
    QTest::qWait(100);
    QCOMPARE(f.decoded.size(), 0);
    // Packet with matching device_id=1 decodes cleanly.
    f.send(2);
    QTRY_COMPARE(f.decoded.size(), 1);
}
