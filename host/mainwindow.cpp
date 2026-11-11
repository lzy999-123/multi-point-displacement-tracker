#include "mainwindow.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSettings>
#include <QCoreApplication>
#include <QSlider>
#include <QSpinBox>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>

#include "videowidget.h"
#include "cameraconfigpath.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    setWindowTitle(tr("多监测点位移跟踪系统"));
    resize(1280, 860);

    auto *toolbar = addToolBar(tr("设备"));
    toolbar->setMovable(false);
    m_discoveryButton = new QPushButton(tr("停止查找"), this);
    m_discoveryStatus = new QLabel(tr("正在查找设备…"), this);
    toolbar->addWidget(m_discoveryButton);
    toolbar->addSeparator();
    toolbar->addWidget(m_discoveryStatus);
    connect(m_discoveryButton, &QPushButton::clicked, this, [this] {
        if (m_discovery.isRunning()) {
            m_discovery.stop();
            m_discoveryButton->setText(tr("自动查找"));
            m_discoveryStatus->setText(tr("已停止查找"));
        } else {
            m_discovery.start();
            m_discoveryButton->setText(tr("停止查找"));
            m_discoveryStatus->setText(tr("正在查找设备…"));
        }
    });

    auto *central = new QWidget(this);
    auto *grid = new QGridLayout(central);
    grid->setContentsMargins(6, 6, 6, 6);
    grid->setSpacing(6);
    for (int i = 0; i < kChannelCount; ++i)
        grid->addWidget(buildChannelPanel(i), i / 2, i % 2);
    grid->addWidget(buildParamPanel(), 1, 1);
    grid->setRowStretch(0, 1);
    grid->setRowStretch(1, 1);
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);
    setCentralWidget(central);

    qRegisterMetaType<DeviceDiscovery::Device>();
    connect(&m_discovery, &DeviceDiscovery::deviceFound, this, &MainWindow::onDeviceFound);
    m_discovery.start();
    statusBar()->showMessage(tr("点击画面中的色块可自动取色"));
}

MainWindow::~MainWindow() = default;

QWidget *MainWindow::buildChannelPanel(int index)
{
    ChannelUi &ui = m_channels[index];
    auto *box = new QGroupBox(tr("监测点 %1").arg(index + 1), this);
    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(4, 4, 4, 4);

    ui.channel = new Channel(index + 1, this);
    ui.video = new VideoWidget(tr("通道 %1").arg(index + 1), box);
    ui.video->setObjectName(QStringLiteral("video-%1").arg(index + 1));
    ui.video->setStatus(tr("未连接"));
    layout->addWidget(ui.video, 1);

    auto *row = new QHBoxLayout;
    ui.urlEdit = new QLineEdit(box);
    ui.urlEdit->setPlaceholderText(tr("http://IP:81/stream（自动发现后填入）"));
    ui.connectButton = new QPushButton(tr("连接"), box);
    auto *rotate = new QCheckBox(tr("旋转180°"), box);
    rotate->setObjectName(QStringLiteral("rotate180-%1").arg(index + 1));
    const QString rotationKey = (index == 0 ? QStringLiteral("camera")
                                           : QStringLiteral("camera%1").arg(index + 1)) + "/rotate180";
    const QString settingsPath = cameraSettingsPath();
    QSettings settings(settingsPath, QSettings::IniFormat);
    rotate->setChecked(settings.value(rotationKey, false).toBool());
    ui.video->setRotate180(rotate->isChecked());
    connect(rotate, &QCheckBox::toggled, ui.video, &VideoWidget::setRotate180);
    connect(rotate, &QCheckBox::toggled, this, [settingsPath, rotationKey](bool checked) {
        QSettings saved(settingsPath, QSettings::IniFormat);
        saved.setValue(rotationKey, checked);
    });
    row->addWidget(ui.urlEdit, 1);
    row->addWidget(rotate);
    row->addWidget(ui.connectButton);
    layout->addLayout(row);

    ui.channel->setParams(ui.params);
    connect(ui.connectButton, &QPushButton::clicked, this, [this, index] { toggleChannel(index); });
    connect(ui.urlEdit, &QLineEdit::returnPressed, this, [this, index] {
        if (!m_channels[index].channel->isOpen())
            toggleChannel(index);
    });
    connect(ui.channel, &Channel::resultReady, this, [this, index](const ChannelResult &r) {
        if (m_channels[index].channel->isOpen()) m_channels[index].video->setResult(r);
        m_channels[index].channel->acknowledgeResult();
    });
    connect(ui.channel, &Channel::stateChanged, this,
            [this, index](MjpegStream::State s, const QString &d) { onStateChanged(index, s, d); });
    connect(ui.channel, &Channel::colorPicked, this, [this, index](const ColorParams &p) {
        m_channels[index].params = p;
        selectParamChannel(index);
        if (m_applyAll->isChecked())
            applyUiParams();
        statusBar()->showMessage(tr("通道 %1 已取色：H %2～%3").arg(index + 1).arg(p.hMin).arg(p.hMax), 4000);
    });
    connect(ui.video, &VideoWidget::imageClicked, this,
            [this, index](int x, int y) { m_channels[index].channel->pickColor(x, y); });
    return box;
}

QWidget *MainWindow::buildParamPanel()
{
    auto *box = new QGroupBox(tr("色块参数"), this);
    auto *form = new QFormLayout(box);

    m_paramChannel = new QComboBox(box);
    for (int i = 0; i < kChannelCount; ++i)
        m_paramChannel->addItem(tr("监测点 %1").arg(i + 1));
    form->addRow(tr("调整对象"), m_paramChannel);

    auto makeSlider = [&](const QString &label, int max) {
        auto *slider = new QSlider(Qt::Horizontal, box);
        slider->setRange(0, max);
        auto *value = new QLabel(box);
        value->setMinimumWidth(30);
        auto *row = new QHBoxLayout;
        row->addWidget(slider, 1);
        row->addWidget(value);
        form->addRow(label, row);
        connect(slider, &QSlider::valueChanged, this, [this, value](int v) {
            value->setNum(v);
            applyUiParams();
        });
        return slider;
    };
    m_hMin = makeSlider(tr("H 下限"), 179);
    m_hMax = makeSlider(tr("H 上限"), 179);
    m_sMin = makeSlider(tr("S 下限"), 255);
    m_sMax = makeSlider(tr("S 上限"), 255);
    m_vMin = makeSlider(tr("V 下限"), 255);
    m_vMax = makeSlider(tr("V 上限"), 255);

    m_minArea = new QSpinBox(box);
    m_minArea->setRange(10, 100000);
    m_minArea->setSingleStep(50);
    m_minArea->setSuffix(tr(" 像素"));
    form->addRow(tr("最小面积"), m_minArea);
    connect(m_minArea, &QSpinBox::valueChanged, this, [this] { applyUiParams(); });

    auto *presets = new QHBoxLayout;
    struct Preset { const char *name; int hMin, hMax; };
    const Preset list[] = {{"红", 170, 10}, {"黄", 20, 35}, {"绿", 40, 80}, {"蓝", 100, 130}};
    for (const Preset &p : list) {
        auto *button = new QPushButton(tr(p.name), box);
        button->setMaximumWidth(48);
        presets->addWidget(button);
        const int hMin = p.hMin, hMax = p.hMax;
        connect(button, &QPushButton::clicked, this, [this, hMin, hMax] { applyPreset(hMin, hMax); });
    }
    presets->addStretch();
    form->addRow(tr("预设颜色"), presets);

    m_applyAll = new QCheckBox(tr("同时应用到 3 个监测点"), box);
    m_showMask = new QCheckBox(tr("显示二值掩膜"), box);
    form->addRow(m_applyAll);
    form->addRow(m_showMask);
    connect(m_applyAll, &QCheckBox::toggled, this, [this](bool on) {
        if (on)
            applyUiParams();
    });
    connect(m_showMask, &QCheckBox::toggled, this, [this](bool on) {
        for (ChannelUi &ui : m_channels) {
            ui.channel->setShowMask(on);
            ui.video->setShowMask(on);
        }
    });

    auto *hint = new QLabel(tr("提示：H 下限大于上限表示跨过 0（红色）。点击画面可直接取色。"), box);
    hint->setWordWrap(true);
    form->addRow(hint);

    connect(m_paramChannel, &QComboBox::currentIndexChanged, this,
            [this](int index) { selectParamChannel(index); });
    selectParamChannel(0);
    return box;
}

void MainWindow::selectParamChannel(int index)
{
    if (index < 0 || index >= kChannelCount)
        return;
    if (m_paramChannel->currentIndex() != index) {
        m_paramChannel->setCurrentIndex(index);  // 会再次进入本函数
        return;
    }
    loadParamsToUi(m_channels[index].params);
}

void MainWindow::loadParamsToUi(const ColorParams &p)
{
    m_loadingParams = true;
    m_hMin->setValue(p.hMin);
    m_hMax->setValue(p.hMax);
    m_sMin->setValue(p.sMin);
    m_sMax->setValue(p.sMax);
    m_vMin->setValue(p.vMin);
    m_vMax->setValue(p.vMax);
    m_minArea->setValue(p.minArea);
    m_loadingParams = false;
}

void MainWindow::applyUiParams()
{
    if (m_loadingParams)
        return;
    ColorParams p = m_channels[m_paramChannel->currentIndex()].params;
    p.hMin = m_hMin->value();
    p.hMax = m_hMax->value();
    p.sMin = m_sMin->value();
    p.sMax = m_sMax->value();
    p.vMin = m_vMin->value();
    p.vMax = m_vMax->value();
    p.minArea = m_minArea->value();

    for (int i = 0; i < kChannelCount; ++i) {
        if (i != m_paramChannel->currentIndex() && !m_applyAll->isChecked())
            continue;
        if (m_channels[i].params != p) {
            m_channels[i].params = p;
            m_channels[i].channel->setParams(p);
        }
    }
}

void MainWindow::applyPreset(int hMin, int hMax)
{
    ColorParams p = m_channels[m_paramChannel->currentIndex()].params;
    p.hMin = hMin;
    p.hMax = hMax;
    p.sMin = 120;
    p.sMax = 255;
    p.vMin = 70;
    p.vMax = 255;
    loadParamsToUi(p);
    applyUiParams();
}

void MainWindow::toggleChannel(int index)
{
    ChannelUi &ui = m_channels[index];
    if (ui.channel->isOpen()) {
        ui.channel->close();
        ui.connectButton->setText(tr("连接"));
        ui.urlEdit->setEnabled(true);
        ui.video->clear();
        ui.video->setStatus(tr("未连接"));
        return;
    }
    const QUrl url = QUrl::fromUserInput(ui.urlEdit->text().trimmed());
    if (!url.isValid() || (url.scheme() != "http" && url.scheme() != "udp") || url.host().isEmpty()) {
        statusBar()->showMessage(tr("通道 %1：地址无效，格式如 udp://192.168.1.101:5006 或 http://192.168.1.101:81/stream").arg(index + 1), 5000);
        return;
    }
    openChannel(index, url);
}

void MainWindow::openChannel(int index, const QUrl &url)
{
    if (index < 0 || index >= kChannelCount) return;
    ChannelUi &ui = m_channels[index];
    ui.automaticConnectionAllowed = true;
    ui.unavailableReason.clear();
    ui.urlEdit->setText(url.toString());
    ui.urlEdit->setEnabled(false);
    ui.connectButton->setText(tr("断开"));
    ui.channel->setParams(ui.params);
    ui.channel->setShowMask(m_showMask->isChecked());
    ui.channel->open(url);
}

void MainWindow::setChannelUnavailable(int index, const QString &reason)
{
    if (index < 0 || index >= kChannelCount) return;
    ChannelUi &ui = m_channels[index];
    ui.automaticConnectionAllowed = false;
    ui.unavailableReason = reason;
    ui.channel->close();
    ui.video->clear();
    ui.video->setStatus(reason);
    ui.connectButton->setText(tr("连接"));
    ui.urlEdit->setEnabled(true);
    ui.deviceAddress.clear();
    updateDiscoveryStatus();
}

void MainWindow::updateDiscoveryStatus()
{
    int found = 0;
    for (const auto &channel : m_channels) found += !channel.deviceAddress.isEmpty();
    m_discoveryStatus->setText(tr("已发现 %1/%2 台设备").arg(found).arg(kChannelCount));
}

void MainWindow::onDeviceFound(const DeviceDiscovery::Device &device)
{
    const int index = device.id - 1;
    if (index < 0 || index >= kChannelCount)
        return;
    ChannelUi &ui = m_channels[index];
    if (!ui.automaticConnectionAllowed) return;
    const QString address = device.address.toString();

    if (!ui.deviceAddress.isEmpty() && ui.deviceAddress != address) {
        // 两台设备报告同一编号：保留先发现的那台
        m_discoveryStatus->setText(tr("编号冲突：%1 和 %2 都是 #%3，请修改固件 DEVICE_ID")
                                       .arg(ui.deviceAddress, address).arg(device.id));
        return;
    }
    if (ui.deviceAddress.isEmpty()) {
        ui.deviceAddress = address;
        updateDiscoveryStatus();
        // 已有设备接入后放慢广播频率（由 2s 改为 10s），彻底避免 2.4GHz 无线广播风暴引起视频卡顿
        m_discovery.setInterval(10000);
    }
    // 用户已手动连接其他地址时不打扰
    if (!ui.channel->isOpen())
        openChannel(index, device.bestStreamUrl());
}

void MainWindow::onStateChanged(int index, MjpegStream::State state, const QString &detail)
{
    QString text;
    switch (state) {
    case MjpegStream::State::Idle:
        text = m_channels[index].automaticConnectionAllowed ? tr("未连接")
                                                            : m_channels[index].unavailableReason;
        break;
    case MjpegStream::State::Connecting:
        text = tr("连接中…");
        break;
    case MjpegStream::State::Streaming:
        text = tr("接收中");
        break;
    case MjpegStream::State::Reconnecting:
        text = tr("重连中：%1").arg(detail);
        break;
    }
    m_channels[index].video->setStatus(text);
}
