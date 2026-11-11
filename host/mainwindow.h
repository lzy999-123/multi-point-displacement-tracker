#pragma once

#include <QMainWindow>

#include <array>

#include "channel.h"
#include "devicediscovery.h"

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class QSpinBox;
class VideoWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    static constexpr int kChannelCount = 3;

    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;
    void openChannel(int index, const QUrl &url);
    void setChannelUnavailable(int index, const QString &reason);

private:
    struct ChannelUi {
        Channel *channel = nullptr;
        VideoWidget *video = nullptr;
        QLineEdit *urlEdit = nullptr;
        QPushButton *connectButton = nullptr;
        ColorParams params;
        QString deviceAddress;  // 通过发现得到的设备 IP，用于识别编号冲突
        bool automaticConnectionAllowed = true;
        QString unavailableReason;
    };

    QWidget *buildChannelPanel(int index);
    QWidget *buildParamPanel();
    void toggleChannel(int index);
    void onDeviceFound(const DeviceDiscovery::Device &device);
    void updateDiscoveryStatus();
    void onStateChanged(int index, MjpegStream::State state, const QString &detail);
    void selectParamChannel(int index);
    void loadParamsToUi(const ColorParams &params);
    void applyUiParams();
    void applyPreset(int hMin, int hMax);

    std::array<ChannelUi, kChannelCount> m_channels;
    DeviceDiscovery m_discovery;
    QPushButton *m_discoveryButton = nullptr;
    QLabel *m_discoveryStatus = nullptr;

    // 参数面板
    QComboBox *m_paramChannel = nullptr;
    QSlider *m_hMin = nullptr, *m_hMax = nullptr;
    QSlider *m_sMin = nullptr, *m_sMax = nullptr;
    QSlider *m_vMin = nullptr, *m_vMax = nullptr;
    QSpinBox *m_minArea = nullptr;
    QCheckBox *m_applyAll = nullptr;
    QCheckBox *m_showMask = nullptr;
    bool m_loadingParams = false;
};
