#include <QtTest>
#include <QNetworkDatagram>
#include <QUdpSocket>
#include <QPushButton>
#include <QSettings>
#include <QTemporaryDir>
#include <array>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "mainwindow.h"
#include "videoprofile.h"

class Camera : public QObject
{
public:
    explicit Camera(int id) : m_id(id) {
        if (!socket.bind(QHostAddress(QStringLiteral("127.0.0.%1").arg(id)), 5006))
            qFatal("Cannot bind test camera");
        connect(&socket, &QUdpSocket::readyRead, this, [this] { receive(); });
        connect(&timer, &QTimer::timeout, this, [this] { frame(); });
    }
    int profile = -1, requests = 0, flags = 0, acknowledgements = 0, repairs = 0;
    QUdpSocket socket;
private:
    void receive() {
        while (socket.hasPendingDatagrams()) {
            const auto datagram = socket.receiveDatagram();
            const auto request = datagram.data();
            if (request.startsWith("VACK")) ++acknowledgements;
            if (request.startsWith("VNCK")) ++repairs;
            if (!request.startsWith("VREQA3") || request.size() != 16) continue;
            client = datagram.senderAddress();
            port = datagram.senderPort();
            profile = quint8(request[12]);
            flags = quint8(request[15]);
            ++requests;
            const auto preset = VideoProfile::preset(unsigned(profile));
            Q_ASSERT(quint8(request[6]) == preset.fps);
            QByteArray reply("VCFG");
            reply.append(char(m_id)); reply.append(char(profile));
            reply.append(request.mid(13, 2));
            for (unsigned value : {unsigned(preset.width), unsigned(preset.height)}) {
                reply.append(char(value)); reply.append(char(value >> 8));
            }
            reply.append(char(preset.fps));
            reply.append(char(0)); reply.append(char(flags)); reply.append(char(0));
            socket.writeDatagram(reply, client, port);
            if (imageProfile != profile) {
                cv::Mat image(preset.height, preset.width, CV_8UC3, cv::Scalar(80, 80, 80));
                cv::circle(image, cv::Point(preset.width / 2, preset.height / 2),
                           preset.height / 6, cv::Scalar(255, 0, 0), cv::FILLED);
                std::vector<uchar> bytes;
                cv::imencode(".jpg", image, bytes);
                jpeg = QByteArray(reinterpret_cast<const char *>(bytes.data()), int(bytes.size()));
                imageProfile = profile;
            }
            timer.start(preset.fps ? 1000 / preset.fps : 33);
            frame();
        }
    }
    void frame() {
        if (!port) return;
        ++sequence;
        const int count = (jpeg.size() + 1399) / 1400;
        for (int index = 0; index < count; ++index) {
            const QByteArray payload = jpeg.mid(index * 1400, 1400);
            const VidPacketHeader header{0x31444956, quint8(m_id), quint8(profile + 1), sequence,
                                         quint8(index), quint8(count), quint16(payload.size()),
                                         quint32(jpeg.size())};
            QByteArray packet(reinterpret_cast<const char *>(&header), sizeof(header));
            packet += payload;
            socket.writeDatagram(packet, client, port);
        }
    }
    int m_id, imageProfile = -1;
    QTimer timer;
    QHostAddress client;
    quint16 port = 0, sequence = 0;
    QByteArray jpeg;
};

class ProfileSwitchTest : public QObject
{
    Q_OBJECT
private slots:
    void threeCameraButtonsSwitchAndPersistProfiles() {
        QTemporaryDir directory;
        QVERIFY(directory.isValid());
        const QString path = directory.filePath("camera.ini");
        std::array<std::unique_ptr<Camera>, 3> cameras;
        for (int i = 0; i < 3; ++i) cameras[i] = std::make_unique<Camera>(i + 1);
        MainWindow window(nullptr, path);
        std::array<QSize, 3> latest{};
        std::array<int, 3> confirmations{};
        for (Channel *channel : window.findChildren<Channel *>()) {
            const int index = channel->id() - 1;
            connect(channel, &Channel::resultReady, &window, [&, index, channel](const ChannelResult &result) {
                if (result.revision == channel->revision()) latest[index] = result.image.size();
            });
            connect(channel, &Channel::videoProfileApplied, &window, [&, index](int) { ++confirmations[index]; });
            window.openChannel(index, QUrl(QStringLiteral("udp://127.0.0.%1:5006").arg(index + 1)));
        }
        window.show();
        const auto allSize = [&](const QSize &size) {
            return latest[0] == size && latest[1] == size && latest[2] == size;
        };
        QTRY_VERIFY_WITH_TIMEOUT(allSize(QSize(320, 240)), 3000);
        QCOMPARE(window.videoProfile(), int(VideoProfile::Qvga25));
        QTRY_VERIFY(confirmations[0] > 0 && confirmations[1] > 0 && confirmations[2] > 0);
        auto *vga = window.findChild<QPushButton *>("profile-vga12");
        auto *qvga = window.findChild<QPushButton *>("profile-qvga25");
        auto *svga = window.findChild<QPushButton *>("profile-svga");
        QVERIFY(vga && qvga && svga);
        QVERIFY(qvga->isChecked());
        latest.fill(QSize());
        vga->click();
        QTRY_VERIFY_WITH_TIMEOUT(allSize(QSize(640, 480)), 3000);
        QVERIFY(vga->isChecked() && !qvga->isChecked());
        latest.fill(QSize());
        qvga->click();
        QTRY_VERIFY_WITH_TIMEOUT(allSize(QSize(320, 240)), 3000);
        QCOMPARE(window.videoProfile(), int(VideoProfile::Qvga25));
        for (const auto &camera : cameras) QCOMPARE(camera->profile, int(VideoProfile::Qvga25));
        QSettings settings(path, QSettings::IniFormat);
        QCOMPARE(settings.value("video/profile").toString(), QStringLiteral("qvga25"));
        for (Channel *channel : window.findChildren<Channel *>()) {
            QCOMPARE(channel->url().scheme(), QStringLiteral("udp"));
            QCOMPARE(videoProfileFromUrl(channel->url()), int(VideoProfile::Qvga25));
        }
        latest.fill(QSize());
        window.setVideoProfile(VideoProfile::Vga12);
        QTRY_VERIFY_WITH_TIMEOUT(allSize(QSize(640, 480)), 3000);
        for (const auto &camera : cameras) QCOMPARE(camera->profile, int(VideoProfile::Vga12));
        QTRY_VERIFY(confirmations[0] >= 3 && confirmations[1] >= 3 && confirmations[2] >= 3);
        latest.fill(QSize());
        svga->click();
        QTRY_VERIFY_WITH_TIMEOUT(allSize(QSize(800, 600)), 3000);
        QVERIFY(svga->isChecked() && !vga->isChecked() && !qvga->isChecked());
        for (const auto &camera : cameras) QCOMPARE(camera->profile, int(VideoProfile::Svga));
        for (Channel *channel : window.findChildren<Channel *>()) channel->close();
        MainWindow reopened(nullptr, path);
        QCOMPARE(reopened.videoProfile(), int(VideoProfile::Svga));
        QVERIFY(reopened.findChild<QPushButton *>("profile-svga")->isChecked());
    }
    void retiredDiagnosticSettingMigratesToQvga() {
        QTemporaryDir directory;
        const QString path = directory.filePath("camera.ini");
        { QSettings settings(path, QSettings::IniFormat); settings.setValue("video/profile", "baseline10"); }
        MainWindow window(nullptr, path);
        QCOMPARE(window.videoProfile(), int(VideoProfile::Qvga25));
        QSettings settings(path, QSettings::IniFormat);
        QCOMPARE(settings.value("video/profile").toString(), QStringLiteral("qvga25"));
    }
    void simpleTransportSwitchesThreeCamerasWithoutFeedback() {
        QTemporaryDir directory;
        const QString path = directory.filePath("camera.ini");
        std::array<std::unique_ptr<Camera>, 3> cameras;
        for (int i = 0; i < 3; ++i) cameras[i] = std::make_unique<Camera>(i + 1);
        MainWindow window(nullptr, path);
        std::array<QSize, 3> sizes{};
        std::array<int, 3> confirmed{};
        for (auto *channel : window.findChildren<Channel *>()) {
            const int index = channel->id() - 1;
            connect(channel, &Channel::resultReady, &window, [&, index, channel](const ChannelResult &result) {
                if (result.revision == channel->revision()) sizes[index] = result.image.size();
            });
            connect(channel, &Channel::videoProfileApplied, &window, [&, index](int) { ++confirmed[index]; });
            window.openChannel(index, QUrl(QStringLiteral("udp://127.0.0.%1:5006?transport=simple").arg(index + 1)));
        }
        window.show();
        for (const auto profile : {VideoProfile::Qvga25, VideoProfile::Vga12,
                                   VideoProfile::Svga, VideoProfile::Qvga25}) {
            window.setVideoProfile(profile);
            sizes.fill({});
            const auto preset = VideoProfile::preset(profile);
            QTRY_VERIFY_WITH_TIMEOUT(sizes[0] == QSize(preset.width, preset.height)
                && sizes[1] == sizes[0] && sizes[2] == sizes[0], 3000);
        }
        QTRY_VERIFY(confirmed[0] >= 4 && confirmed[1] >= 4 && confirmed[2] >= 4);
        for (const auto &camera : cameras) {
            QCOMPARE(camera->flags, 3);
            QCOMPARE(camera->acknowledgements, 0);
            QCOMPARE(camera->repairs, 0);
        }
        for (auto *channel : window.findChildren<Channel *>()) channel->close();
    }
};

QTEST_MAIN(ProfileSwitchTest)
#include "profile_switch_test.moc"
