#pragma once

#include <QMainWindow>
#include <QTimer>

#include <array>

#include "channel.h"
#include "devicediscovery.h"
#include "artifactmonitor.h"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSlider;
class QSpinBox;
class VideoWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    static constexpr int kChannelCount = 3;

    explicit MainWindow(QWidget *parent = nullptr, const QString &settingsPath = {});
    ~MainWindow() override;
    void openChannel(int index, const QUrl &url);
    void setChannelUnavailable(int index, const QString &reason);
    void setVideoProfile(int profile);
    void setJpegQualityMode(int qualityMode);
    int videoProfile() const { return m_videoProfile; }

private:
    struct ChannelUi {
        QGroupBox *panelBox = nullptr;
        Channel *channel = nullptr;
        VideoWidget *video = nullptr;
        QPushButton *connectButton = nullptr;
        QPushButton *calibrateButton = nullptr;
        QLabel *cardBadge = nullptr;
        QLabel *cardDetail = nullptr;
        QUrl lastUrl;
        ColorParams params;
        AutoContourParams autoParams;
        AnomalyMetrics lastAnomaly;
        bool hasTarget = false;
        bool baselineSet = false;
        bool streamAvailable = false;
        double lastFps = 0.0;
        QSize lastImageSize;
        qint64 observedAtMs = -1;
        QString deviceAddress;  // 通过发现得到的设备 IP，用于识别编号冲突
        bool automaticConnectionAllowed = true;
        QString unavailableReason;
    };

    QWidget *buildChannelPanel(int index);
    QWidget *buildMonitorDashboard();
    void toggleChannel(int index);
    void onDeviceFound(const DeviceDiscovery::Device &device);
    void updateDiscoveryStatus();
    void onStateChanged(int index, MjpegStream::State state, const QString &detail);

    // 藏品监测与多机位融合操作
    void calibrateAllBaselines();
    void resetAllBaselines();
    void calibrateSingleChannel(int index);
    void applyMonitorThresholds();
    void applyDetectorMode(int modeIndex);
    void updateFusionDecision();
    void appendLog(const QString &text);
    void exportEventLog();
    void saveAlertSnapshot(const QString &tag);

    QString m_settingsPath;
    int m_videoProfile = 0;
    int m_jpegQualityMode = 30; // 10..40 = 锁定指定硬件JPEG质量，-1 = 自适应18~42
    QComboBox *m_comboQuality = nullptr;
    QLabel *m_profileLabel = nullptr;
    std::array<ChannelUi, kChannelCount> m_channels;
    DeviceDiscovery m_discovery;
    QPushButton *m_discoveryButton = nullptr;
    QLabel *m_discoveryStatus = nullptr;

    // 监控仪表盘控件
    QPushButton *m_btnCalibAll = nullptr;
    QPushButton *m_btnResetAll = nullptr;
    QDoubleSpinBox *m_spinTiltThresh = nullptr;
    QSpinBox *m_spinScaleThresh = nullptr;
    QDoubleSpinBox *m_spinDispThresh = nullptr;
    QSpinBox *m_spinMinArea = nullptr;
    QComboBox *m_comboMode = nullptr;
    QCheckBox *m_chkShowMask = nullptr;
    QCheckBox *m_chkEnhance = nullptr;
    QCheckBox *m_chkBeep = nullptr;
    QCheckBox *m_chkAutoSnap = nullptr;

    // 多机位融合看板
    QLabel *m_lblFusionStatus = nullptr;
    QLabel *m_lblFusionDetail = nullptr;
    QPlainTextEdit *m_txtEventLog = nullptr;
    QTimer m_monitorTimer;

    MonitorThresholds m_thresholds;
    QString m_lastReportedSignature;
    qint64 m_lastBeepMs = -10000;
    qint64 m_lastSnapshotMs = -10000;
};
