#include "mainwindow.h"

#include <cmath>
#include <QApplication>
#include <QCheckBox>
#include <QButtonGroup>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSettings>
#include <QSpinBox>
#include <QStatusBar>
#include <QTabWidget>
#include <QToolBar>
#include <QVBoxLayout>

#include "cameraconfigpath.h"
#include "videowidget.h"
#include "videoprofile.h"

MainWindow::MainWindow(QWidget *parent, const QString &settingsPath)
    : QMainWindow(parent), m_settingsPath(settingsPath.isEmpty() ? cameraSettingsPath() : settingsPath)
{
    QSettings settings(m_settingsPath, QSettings::IniFormat);
    const QString savedProfile = settings.value(QStringLiteral("video/profile")).toString();
    m_videoProfile = savedProfile == QStringLiteral("svga") ? VideoProfile::Svga
        : savedProfile == QStringLiteral("vga12") ? VideoProfile::Vga12 : VideoProfile::Qvga25;
    settings.setValue(QStringLiteral("video/profile"), videoProfileKey(m_videoProfile));
    const int savedQuality = settings.value(QStringLiteral("video/jpegQuality"), 30).toInt();
    m_jpegQualityMode = (savedQuality == -1 || (savedQuality >= 10 && savedQuality <= 40)) ? savedQuality : 30;
    settings.setValue(QStringLiteral("video/jpegQuality"), m_jpegQualityMode);
    setWindowTitle(tr("多检测点特征跟踪系统（ESP32 三视角无线图传）"));

    auto *toolbar = addToolBar(tr("设备与控制"));
    toolbar->setMovable(false);
    m_discoveryButton = new QPushButton(tr("停止查找"), this);
    m_discoveryStatus = new QLabel(tr("正在查找设备…"), this);
    toolbar->addWidget(m_discoveryButton);
    toolbar->addSeparator();
    toolbar->addWidget(m_discoveryStatus);
    toolbar->addSeparator();
    auto *profiles = new QButtonGroup(this);
    for (const auto profile : {VideoProfile::Qvga25, VideoProfile::Vga12, VideoProfile::Svga}) {
        const auto preset = VideoProfile::preset(profile);
        auto *button = new QPushButton(tr("%1×%2").arg(preset.width).arg(preset.height), this);
        button->setToolTip(videoProfileCadence(profile));
        button->setObjectName(QStringLiteral("profile-") + videoProfileKey(profile));
        button->setCheckable(true);
        button->setChecked(profile == m_videoProfile);
        profiles->addButton(button, profile);
        toolbar->addWidget(button);
        connect(button, &QPushButton::clicked, this, [this, profile] { setVideoProfile(profile); });
    }
    m_comboQuality = new QComboBox(this);
    m_comboQuality->setObjectName(QStringLiteral("jpeg-quality-mode"));
    m_comboQuality->setToolTip(tr("OV3660 硬件 JPEG 质量：数值越小画质越清晰（体积越大）"));
    m_comboQuality->addItem(tr("JPEG 锁定 30"), 30);
    m_comboQuality->addItem(tr("JPEG 锁定 24（更清）"), 24);
    m_comboQuality->addItem(tr("JPEG 锁定 20（高清）"), 20);
    m_comboQuality->addItem(tr("JPEG 锁定 16（超清）"), 16);
    m_comboQuality->addItem(tr("JPEG 自动 18～42"), -1);
    const int qIdx = m_comboQuality->findData(m_jpegQualityMode);
    m_comboQuality->setCurrentIndex(qIdx >= 0 ? qIdx : 0);
    connect(m_comboQuality, &QComboBox::currentIndexChanged, this, [this](int idx) {
        if (idx >= 0) setJpegQualityMode(m_comboQuality->itemData(idx).toInt());
    });
    toolbar->addSeparator();
    toolbar->addWidget(m_comboQuality);

    const auto startupPreset = VideoProfile::preset(unsigned(m_videoProfile));
    const QString qDesc = m_jpegQualityMode > 0
        ? tr("锁定q=%1").arg(m_jpegQualityMode)
        : tr("自动q=18～42");
    m_profileLabel = new QLabel(tr("UDP：%1×%2 · %3")
        .arg(startupPreset.width).arg(startupPreset.height).arg(qDesc), this);
    m_profileLabel->setObjectName(QStringLiteral("video-profile"));
    m_profileLabel->setToolTip(tr("同时切换三路相机；实际接收和显示速度由采集与网络决定"));
    toolbar->addWidget(m_profileLabel);
    toolbar->addSeparator();

    m_chkCrossViewSync = new QCheckBox(tr("三路联动锁定"), this);
    m_chkCrossViewSync->setObjectName(QStringLiteral("cross-view-sync"));
    m_chkCrossViewSync->setToolTip(tr("开启后，在任意一路框选目标，其余两路自动按外观特征全图搜索并同步锁定；关闭则三路独立框选"));
    m_chkCrossViewSync->setChecked(settings.value(QStringLiteral("tracking/crossViewSync"), true).toBool());
    connect(m_chkCrossViewSync, &QCheckBox::toggled, this, [this](bool checked) {
        QSettings saved(m_settingsPath, QSettings::IniFormat);
        saved.setValue(QStringLiteral("tracking/crossViewSync"), checked);
        appendLog(checked ? tr("已开启三路联动锁定：任一路框选目标将自动同步至其余检测点")
                          : tr("已切换为三路独立框选模式"));
    });
    toolbar->addWidget(m_chkCrossViewSync);

    auto *btnClearAll = new QPushButton(tr("清除全部"), this);
    btnClearAll->setToolTip(tr("一键清除所有检测点的锁定目标与运动轨迹"));
    connect(btnClearAll, &QPushButton::clicked, this, &MainWindow::clearAllTargets);
    toolbar->addWidget(btnClearAll);

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
    for (int i = 0; i < kChannelCount; ++i) {
        grid->addWidget(buildChannelPanel(i), i / 2, i % 2);
    }
    grid->addWidget(buildMonitorDashboard(), 1, 1);
    grid->setSizeConstraint(QLayout::SetFixedSize);
    setCentralWidget(central);

    qRegisterMetaType<DeviceDiscovery::Device>();
    connect(&m_discovery, &DeviceDiscovery::deviceFound, this, &MainWindow::onDeviceFound);
    // 读取上次发现记录的 IP 作为单播探测提示（不跳过 ID 核对，不依赖 COM 口号）
    for (int i = 0; i < kChannelCount; ++i) {
        const QString group = (i == 0 ? QStringLiteral("camera") : QStringLiteral("camera%1").arg(i + 1));
        QHostAddress hint(settings.value(group + QStringLiteral("/lastIp")).toString().trimmed());
        if (hint.isNull()) {
            const QUrl oldUrl(settings.value(group + QStringLiteral("/url")).toString().trimmed());
            if (oldUrl.isValid()) hint = QHostAddress(oldUrl.host());
        }
        if (hint.protocol() == QAbstractSocket::IPv4Protocol && !hint.isNull())
            m_discovery.setTarget(i + 1, hint);
    }
    m_discovery.start();

    // 默认应用门限
    applyMonitorThresholds();
    m_monitorTimer.setInterval(100);
    connect(&m_monitorTimer, &QTimer::timeout, this, &MainWindow::updateFusionDecision);
    m_monitorTimer.start();
    updateFusionDecision();
    statusBar()->showMessage(tr("操作说明：左键在任意一路拖拽框选目标，三路自动联动锁定并实时跟踪（右键清除单路，顶栏可开关联动）"));
    appendLog(tr("多检测点特征跟踪系统就绪（三路联动锁定已开启）：在任意一路左键框选目标，其余两路将自动按特征同步锁定。"));
    setFixedSize(sizeHint());
}

MainWindow::~MainWindow() = default;

void MainWindow::setVideoProfile(int profile)
{
    if (!VideoProfile::valid(unsigned(profile)) || profile == m_videoProfile) return;
    m_videoProfile = profile;
    for (const auto candidate : {VideoProfile::Vga12, VideoProfile::Qvga25, VideoProfile::Svga})
        if (auto *button = findChild<QPushButton *>(QStringLiteral("profile-") + videoProfileKey(candidate)))
            button->setChecked(candidate == profile);
    const auto profilePreset = VideoProfile::preset(unsigned(profile));
    const QString qDesc = m_jpegQualityMode > 0
        ? tr("锁定q=%1").arg(m_jpegQualityMode)
        : tr("自动q=18～42");
    m_profileLabel->setText(tr("UDP：%1×%2 · %3")
        .arg(profilePreset.width).arg(profilePreset.height).arg(qDesc));
    QSettings settings(m_settingsPath, QSettings::IniFormat);
    settings.setValue(QStringLiteral("video/profile"), videoProfileKey(profile));
    resetAllBaselines();
    for (int i = 0; i < kChannelCount; ++i) {
        auto &ui = m_channels[i];
        ui.video->clear();
        if (ui.channel->isOpen())
            openChannel(i, withVideoProfile(ui.channel->url(), profile));
    }
    const auto preset = VideoProfile::preset(unsigned(profile));
    appendLog(tr("切换全部机位至 %1×%2 / %3（UDP），请重新锁定基准")
                  .arg(preset.width).arg(preset.height).arg(qDesc));
    statusBar()->showMessage(tr("正在切换相机分辨率；画面恢复后请重新锁定基准"), 6000);
}

void MainWindow::setJpegQualityMode(int qualityMode)
{
    if (qualityMode != -1 && (qualityMode < 10 || qualityMode > 40)) return;
    if (qualityMode == m_jpegQualityMode) return;
    m_jpegQualityMode = qualityMode;
    QSettings settings(m_settingsPath, QSettings::IniFormat);
    settings.setValue(QStringLiteral("video/jpegQuality"), m_jpegQualityMode);
    const auto preset = VideoProfile::preset(unsigned(m_videoProfile));
    const QString qDesc = m_jpegQualityMode > 0
        ? tr("锁定q=%1").arg(m_jpegQualityMode)
        : tr("自动q=18～42");
    if (m_profileLabel) {
        m_profileLabel->setText(tr("UDP：%1×%2 · %3")
            .arg(preset.width).arg(preset.height).arg(qDesc));
    }
    for (int i = 0; i < kChannelCount; ++i) {
        auto &ui = m_channels[i];
        if (!ui.channel->isOpen()) continue;
        QUrl url = ui.channel->url();
        if (!QHostAddress(url.host()).isLoopback()) {
            QUrlQuery query(url);
            query.removeAllQueryItems(QStringLiteral("hqq"));
            query.addQueryItem(QStringLiteral("hqq"), QString::number(m_jpegQualityMode));
            url.setQuery(query);
            openChannel(i, url);
        }
    }
    appendLog(tr("已切换无线 JPEG 质量模式：%1（OV3660 数值越小越清晰）").arg(qDesc));
}

QWidget *MainWindow::buildChannelPanel(int index)
{
    ChannelUi &ui = m_channels[index];
    auto *box = new QGroupBox(tr("检测点 %1（ESP32 无线）").arg(index + 1), this);
    ui.panelBox = box;
    box->setFixedSize(560, 418);
    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(6, 4, 6, 6);
    layout->setSpacing(4);

    ui.channel = new Channel(index + 1, this);
    ui.channel->setDetectorMode(ChannelWorker::AutoContour);
    ui.video = new VideoWidget(tr("检测点 %1 视角").arg(index + 1), box);
    ui.video->setObjectName(QStringLiteral("video-%1").arg(index + 1));
    ui.video->setStatus(tr("未连接"));
    layout->addWidget(ui.video, 0, Qt::AlignHCenter);

    auto *row = new QHBoxLayout;
    row->setContentsMargins(24, 0, 24, 0);
    row->setSpacing(8);

    auto *rotate = new QCheckBox(tr("旋转180°"), box);
    rotate->setObjectName(QStringLiteral("rotate180-%1").arg(index + 1));
    const QString rotationKey = (index == 0 ? QStringLiteral("camera")
                                            : QStringLiteral("camera%1").arg(index + 1)) + "/rotate180";
    const QString settingsPath = m_settingsPath;
    QSettings settings(settingsPath, QSettings::IniFormat);
    rotate->setChecked(settings.value(rotationKey, false).toBool());
    ui.video->setRotate180(rotate->isChecked());
    connect(rotate, &QCheckBox::toggled, ui.video, &VideoWidget::setRotate180);
    connect(rotate, &QCheckBox::toggled, this, [settingsPath, rotationKey](bool checked) {
        QSettings saved(settingsPath, QSettings::IniFormat);
        saved.setValue(rotationKey, checked);
    });

    ui.clearButton = new QPushButton(tr("清除"), box);
    ui.clearButton->setToolTip(tr("清除当前检测点锁定目标与轨迹（也可直接在画面右键清除）"));
    connect(ui.clearButton, &QPushButton::clicked, this, [this, index] {
        clearSingleChannelTarget(index);
    });

    ui.calibrateButton = new QPushButton(tr("📷 标定"), box);
    ui.calibrateButton->setToolTip(tr("锁定当前特征目标为基准形态，开启异动告警"));
    connect(ui.calibrateButton, &QPushButton::clicked, this, [this, index] {
        calibrateSingleChannel(index);
    });

    ui.connectButton = new QPushButton(tr("连接"), box);

    row->addWidget(rotate);
    row->addStretch(1);
    row->addWidget(ui.clearButton);
    row->addWidget(ui.calibrateButton);
    row->addWidget(ui.connectButton);
    layout->addLayout(row);

    connect(ui.connectButton, &QPushButton::clicked, this, [this, index] { toggleChannel(index); });

    connect(ui.channel, &Channel::resultReady, this, [this, index](const ChannelResult &r) {
        const auto preset = VideoProfile::preset(unsigned(m_videoProfile));
        const qint64 age = monitorNowMs() - r.observedAtMs;
        if (m_channels[index].channel->isOpen() && r.revision == m_channels[index].channel->revision()
            && r.observedAtMs >= 0 && age >= 0 && age <= kMonitorFreshnessMs
            && r.image.size() == QSize(preset.width, preset.height)) {
            auto &ui = m_channels[index];
            ui.video->setResult(r);
            ui.lastAnomaly = r.anomaly;
            ui.hasTarget = r.hasTarget;
            ui.trackingActive = r.trackingActive;
            ui.baselineSet = r.baselineCalibrated;
            ui.lastTargetCenter = r.targetCenter;
            ui.lastDeltaX = r.deltaX;
            ui.lastDeltaY = r.deltaY;
            ui.lastDisplacement = r.displacement;
            ui.lastSpeed = r.speedPxPerSec;
            ui.lastScaleRatio = r.scaleRatio;
            ui.observedAtMs = r.observedAtMs;
            ui.streamAvailable = true;
            ui.lastFps = r.fps;
            ui.lastImageSize = r.image.size();
            if (ui.panelBox) {
                ui.panelBox->setTitle(tr("检测点 %1（ESP32 无线 · %2×%3 · %4 fps）")
                                          .arg(index + 1)
                                          .arg(r.image.width()).arg(r.image.height())
                                          .arg(r.fps, 0, 'f', 1));
            }
            updateFusionDecision();
        }
        m_channels[index].channel->acknowledgeResult();
    });

    connect(ui.channel, &Channel::stateChanged, this,
            [this, index](MjpegStream::State s, const QString &d) { onStateChanged(index, s, d); });
    connect(ui.channel, &Channel::activeStreamChanged, this, [this, index](const QUrl &url) {
        m_channels[index].lastUrl = url;
    });
    connect(ui.channel, &Channel::videoProfileApplied, this, [this, index](int profile) {
        if (profile != m_videoProfile) return;
        const auto preset = VideoProfile::preset(unsigned(profile));
        appendLog(tr("检测点 %1 已确认：%2×%3，%4，UDP")
                      .arg(index + 1).arg(preset.width).arg(preset.height).arg(videoProfileCadence(profile)));
    });
    connect(ui.channel, &Channel::calibrationFinished, this,
            [this, index](quint64 revision, bool success, const QString &reason) {
        if (revision != m_channels[index].channel->revision()) return;
        appendLog(tr("检测点 %1 基准%2：%3").arg(index + 1)
                      .arg(success ? tr("锁定完成") : tr("未锁定")).arg(reason));
        statusBar()->showMessage(reason, 4000);
    });
    connect(ui.channel, &Channel::targetSignatureExtracted, this,
            [this, index](quint64 revision, const TargetSignature &sig) {
        if (revision != m_channels[index].channel->revision()) return;
        broadcastTargetSignature(index, sig);
    });

    connect(ui.video, &VideoWidget::clearRequested, this, [this, index] {
        clearSingleChannelTarget(index);
    });

    connect(ui.video, &VideoWidget::imageClicked, this, [this, index](int x, int y) {
        auto &ui = m_channels[index];
        if (!ui.streamAvailable || ui.observedAtMs < 0
            || monitorNowMs() - ui.observedAtMs > kMonitorFreshnessMs) {
            statusBar()->showMessage(tr("画面已过期，请等待新画面再选择目标"), 3000);
            return;
        }
        ui.baselineSet = false;
        ui.lastAnomaly = {};
        ui.trackingActive = true;
        m_channels[index].channel->pickTarget(x, y);
        updateFusionDecision();
        statusBar()->showMessage(tr("检测点 %1：已点选 (%2, %3)，实时跟踪已启动（右键可清除）")
                                    .arg(index + 1).arg(x).arg(y), 4000);
    });

    connect(ui.video, &VideoWidget::roiSelected, this, [this, index](const QRect &roi) {
        auto &ui = m_channels[index];
        if (!ui.streamAvailable || ui.observedAtMs < 0
            || monitorNowMs() - ui.observedAtMs > kMonitorFreshnessMs) {
            statusBar()->showMessage(tr("画面已过期，请等待新画面再框选目标"), 3000);
            return;
        }
        ui.baselineSet = false;
        ui.lastAnomaly = {};
        ui.trackingActive = true;
        m_channels[index].channel->selectTargetRoi(roi);
        updateFusionDecision();
        appendLog(tr("检测点 %1 已锁定特征目标 (%2, %3, %4×%5)，CSRT 多特征实时跟踪已启动")
                      .arg(index + 1).arg(roi.x()).arg(roi.y()).arg(roi.width()).arg(roi.height()));
        statusBar()->showMessage(tr("检测点 %1：已锁定目标 (%2×%3)，实时跟踪中（右键清除，点【标定】可启用告警）")
                                    .arg(index + 1).arg(roi.width()).arg(roi.height()), 5000);
    });

    return box;
}

QWidget *MainWindow::buildMonitorDashboard()
{
    auto *box = new QGroupBox(tr("多检测点跟踪状态与控制"), this);
    box->setFixedSize(560, 418);
    auto *rootLayout = new QVBoxLayout(box);
    rootLayout->setContentsMargins(6, 4, 6, 6);
    rootLayout->setSpacing(4);

    auto *tabs = new QTabWidget(box);
    rootLayout->addWidget(tabs);

    // ==================== 标签 1：实时跟踪与监控（主视窗） ====================
    auto *monitorTab = new QWidget(tabs);
    auto *monLayout = new QVBoxLayout(monitorTab);
    monLayout->setContentsMargins(6, 6, 6, 6);
    monLayout->setSpacing(6);

    // 1. 总览状态横幅
    m_lblFusionStatus = new QLabel(tr("⚪ 系统待命：左键框选目标即开始实时跟踪"), monitorTab);
    m_lblFusionStatus->setObjectName(QStringLiteral("fusion-status"));
    m_lblFusionStatus->setWordWrap(true);
    m_lblFusionStatus->setFixedHeight(44);
    m_lblFusionStatus->setAlignment(Qt::AlignCenter);
    m_lblFusionStatus->setStyleSheet(QStringLiteral(
        "background-color: #546e7a; color: #ffffff; font-weight: bold; font-size: 10pt; padding: 4px 8px; border-radius: 5px;"));
    monLayout->addWidget(m_lblFusionStatus);

    m_lblFusionDetail = new QLabel(tr("三路检测点就绪"), monitorTab);
    m_lblFusionDetail->setObjectName(QStringLiteral("fusion-detail"));
    m_lblFusionDetail->setVisible(false);

    // 2. 三路检测点独立状态卡片（实时坐标、位移、速度与异常状态）
    auto *cardsLayout = new QVBoxLayout;
    cardsLayout->setSpacing(4);
    for (int i = 0; i < kChannelCount; ++i) {
        auto *card = new QWidget(monitorTab);
        card->setFixedHeight(34);
        card->setStyleSheet(QStringLiteral(
            "QWidget { background-color: #f3f5f8; border: 1px solid #dce2ea; border-radius: 4px; }"));
        auto *row = new QHBoxLayout(card);
        row->setContentsMargins(8, 3, 8, 3);
        row->setSpacing(6);

        auto *nameLbl = new QLabel(tr("检测点%1").arg(i + 1), card);
        nameLbl->setFixedWidth(48);
        nameLbl->setStyleSheet(QStringLiteral("border: none; background: transparent; font-weight: bold; color: #263238;"));

        auto *badge = new QLabel(tr("未连接"), card);
        badge->setFixedSize(82, 22);
        badge->setAlignment(Qt::AlignCenter);
        badge->setStyleSheet(QStringLiteral(
            "border: none; background-color: #78909c; color: white; font-weight: bold; font-size: 8.5pt; border-radius: 3px;"));

        auto *detail = new QLabel(tr("等待 ESP32 无线连接…"), card);
        detail->setStyleSheet(QStringLiteral("border: none; background: transparent; color: #37474f; font-size: 8.5pt;"));

        row->addWidget(nameLbl);
        row->addWidget(badge);
        row->addWidget(detail, 1);
        cardsLayout->addWidget(card);

        m_channels[i].cardBadge = badge;
        m_channels[i].cardDetail = detail;
    }
    monLayout->addLayout(cardsLayout);

    // 3. 快捷操作条
    auto *actionRow = new QHBoxLayout;
    actionRow->setSpacing(6);
    m_btnCalibAll = new QPushButton(tr("📷 锁定基准"), monitorTab);
    m_btnCalibAll->setToolTip(tr("将三路当前跟踪目标锁定为基准形态，启用多视角异常告警"));
    m_btnCalibAll->setStyleSheet(QStringLiteral(
        "background-color: #0288d1; color: white; font-weight: bold; padding: 5px 10px; border-radius: 4px;"));
    auto *btnZeroTrack = new QPushButton(tr("轨迹归零"), monitorTab);
    btnZeroTrack->setToolTip(tr("将当前目标位置设为位移原点，并清空运动轨迹尾迹"));
    btnZeroTrack->setStyleSheet(QStringLiteral("padding: 5px 8px;"));
    m_btnResetAll = new QPushButton(tr("重置基准"), monitorTab);
    m_btnResetAll->setStyleSheet(QStringLiteral("padding: 5px 8px;"));
    auto *btnExportLog = new QPushButton(tr("导出日志"), monitorTab);
    btnExportLog->setToolTip(tr("导出运行与告警事件日志"));
    btnExportLog->setStyleSheet(QStringLiteral("padding: 5px 8px;"));
    auto *btnClearLog = new QPushButton(tr("清空"), monitorTab);
    btnClearLog->setMaximumWidth(52);
    btnClearLog->setStyleSheet(QStringLiteral("padding: 5px 6px;"));
    actionRow->addWidget(m_btnCalibAll, 2);
    actionRow->addWidget(btnZeroTrack, 1);
    actionRow->addWidget(m_btnResetAll, 1);
    actionRow->addWidget(btnExportLog);
    actionRow->addWidget(btnClearLog);
    monLayout->addLayout(actionRow);

    connect(m_btnCalibAll, &QPushButton::clicked, this, &MainWindow::calibrateAllBaselines);
    connect(btnZeroTrack, &QPushButton::clicked, this, &MainWindow::resetAllTrajectories);
    connect(m_btnResetAll, &QPushButton::clicked, this, &MainWindow::resetAllBaselines);
    connect(btnExportLog, &QPushButton::clicked, this, &MainWindow::exportEventLog);
    connect(btnClearLog, &QPushButton::clicked, this, [this] { m_txtEventLog->clear(); });

    // 4. 事件与跟踪记录（占据底部主要空间，清晰易读）
    m_txtEventLog = new QPlainTextEdit(monitorTab);
    m_txtEventLog->setReadOnly(true);
    m_txtEventLog->setPlaceholderText(tr("多检测点目标跟踪、位移/尺度变化、丢失重找与告警事件将在此记录…"));
    m_txtEventLog->setStyleSheet(QStringLiteral(
        "background-color: #1c2026; color: #cfd8dc; font-family: Consolas, 'Microsoft YaHei UI', monospace; font-size: 8.5pt; border-radius: 4px;"));
    monLayout->addWidget(m_txtEventLog, 1);

    tabs->addTab(monitorTab, tr("实时跟踪与监控"));

    // ==================== 标签 2：参数设置（收起不占视线） ====================
    auto *settingsTab = new QWidget(tabs);
    auto *setLayout = new QVBoxLayout(settingsTab);
    setLayout->setContentsMargins(10, 8, 10, 8);
    setLayout->setSpacing(6);

    auto *form = new QFormLayout;
    form->setSpacing(6);

    m_spinTiltThresh = new QDoubleSpinBox(settingsTab);
    m_spinTiltThresh->setRange(1.0, 45.0);
    m_spinTiltThresh->setSingleStep(0.5);
    m_spinTiltThresh->setValue(m_thresholds.tiltThresh);
    m_spinTiltThresh->setSuffix(tr(" °"));

    m_spinScaleThresh = new QSpinBox(settingsTab);
    m_spinScaleThresh->setRange(5, 50);
    m_spinScaleThresh->setSingleStep(1);
    m_spinScaleThresh->setValue(int(m_thresholds.scaleThresh * 100.0));
    m_spinScaleThresh->setSuffix(tr(" %"));

    m_spinDispThresh = new QDoubleSpinBox(settingsTab);
    m_spinDispThresh->setRange(2.0, 100.0);
    m_spinDispThresh->setSingleStep(1.0);
    m_spinDispThresh->setValue(m_thresholds.dispThresh);
    m_spinDispThresh->setSuffix(tr(" px"));

    m_spinMinArea = new QSpinBox(settingsTab);
    m_spinMinArea->setRange(50, 100000);
    m_spinMinArea->setSingleStep(50);
    m_spinMinArea->setValue(300);
    m_spinMinArea->setSuffix(tr(" 像素"));

    m_comboMode = new QComboBox(settingsTab);
    m_comboMode->addItem(tr("CSRT 多特征跟踪（推荐：框选即跟、防串色）"));
    m_comboMode->addItem(tr("HSV 颜色色块 (向后兼容)"));

    m_chkShowTrajectory = new QCheckBox(tr("在画面中显示目标实时运动轨迹"), settingsTab);
    m_chkShowTrajectory->setChecked(true);
    m_chkShowMask = new QCheckBox(tr("在画面中显示处理掩膜 / 轮廓边缘"), settingsTab);
    m_chkEnhance = new QCheckBox(tr("画面清晰度增强（去灰 / 轻锐化）"), settingsTab);
    m_chkEnhance->setChecked(true);

    form->addRow(tr("倾斜报警门限"), m_spinTiltThresh);
    form->addRow(tr("尺寸突变门限"), m_spinScaleThresh);
    form->addRow(tr("位移偏离门限"), m_spinDispThresh);
    form->addRow(tr("最小目标面积"), m_spinMinArea);
    form->addRow(tr("检测算法模式"), m_comboMode);
    setLayout->addLayout(form);
    setLayout->addWidget(m_chkShowTrajectory);
    setLayout->addWidget(m_chkShowMask);
    setLayout->addWidget(m_chkEnhance);
    setLayout->addStretch();

    tabs->addTab(settingsTab, tr("参数设置"));

    connect(m_spinTiltThresh, &QDoubleSpinBox::valueChanged, this, &MainWindow::applyMonitorThresholds);
    connect(m_spinScaleThresh, &QSpinBox::valueChanged, this, &MainWindow::applyMonitorThresholds);
    connect(m_spinDispThresh, &QDoubleSpinBox::valueChanged, this, &MainWindow::applyMonitorThresholds);
    connect(m_spinMinArea, &QSpinBox::valueChanged, this, [this](int area) {
        for (auto &c : m_channels) {
            c.autoParams.minArea = area;
            c.channel->setAutoParams(c.autoParams);
        }
    });
    connect(m_comboMode, &QComboBox::currentIndexChanged, this, &MainWindow::applyDetectorMode);
    connect(m_chkShowTrajectory, &QCheckBox::toggled, this, [this](bool show) {
        for (auto &c : m_channels) {
            c.video->setShowTrajectory(show);
        }
    });
    connect(m_chkShowMask, &QCheckBox::toggled, this, [this](bool show) {
        for (auto &c : m_channels) {
            c.channel->setShowMask(show);
            c.video->setShowMask(show);
        }
    });
    connect(m_chkEnhance, &QCheckBox::toggled, this, [this](bool enable) {
        for (auto &c : m_channels) {
            c.channel->setImageEnhance(enable);
        }
    });

    return box;
}

void MainWindow::calibrateAllBaselines()
{
    int count = 0;
    for (int i = 0; i < kChannelCount; ++i) {
        if (m_channels[i].channel->isOpen() && m_channels[i].streamAvailable
            && m_channels[i].observedAtMs >= 0
            && monitorNowMs() - m_channels[i].observedAtMs <= kMonitorFreshnessMs) {
            m_channels[i].baselineSet = false;
            m_channels[i].channel->calibrateBaseline();
            ++count;
        }
    }
    appendLog(tr("已向 %1 路在线检测点请求锁定基准；等待各路确认").arg(count));
    statusBar()->showMessage(tr("基准请求已发送；具体成功/失败见事件记录"), 5000);
    updateFusionDecision();
}

void MainWindow::resetAllBaselines()
{
    for (int i = 0; i < kChannelCount; ++i) {
        m_channels[i].channel->resetBaseline();
        m_channels[i].baselineSet = false;
        m_channels[i].lastAnomaly = AnomalyMetrics();
    }
    m_lastReportedSignature.clear();
    appendLog(tr("已清除所有检测点的基准参考数据（保留当前实时跟踪）"));
    updateFusionDecision();
}

void MainWindow::resetAllTrajectories()
{
    for (int i = 0; i < kChannelCount; ++i) {
        m_channels[i].channel->resetTrajectoryOrigin();
        m_channels[i].lastDeltaX = 0.0;
        m_channels[i].lastDeltaY = 0.0;
        m_channels[i].lastDisplacement = 0.0;
        m_channels[i].lastSpeed = 0.0;
    }
    appendLog(tr("已归零所有检测点的位移原点并清空轨迹记录"));
    statusBar()->showMessage(tr("位移原点与轨迹尾迹已归零"), 3000);
    updateFusionDecision();
}

void MainWindow::calibrateSingleChannel(int index)
{
    if (index >= 0 && index < kChannelCount) {
        if (!m_channels[index].streamAvailable || m_channels[index].observedAtMs < 0
            || monitorNowMs() - m_channels[index].observedAtMs > kMonitorFreshnessMs) {
            appendLog(tr("检测点 %1 无近期新画面，未锁定基准").arg(index + 1));
            return;
        }
        m_channels[index].baselineSet = false;
        m_channels[index].channel->calibrateBaseline();
        appendLog(tr("检测点 %1 已请求锁定基准，等待确认").arg(index + 1));
        updateFusionDecision();
    }
}

void MainWindow::clearSingleChannelTarget(int index)
{
    if (index < 0 || index >= kChannelCount) return;
    auto &ui = m_channels[index];
    ui.channel->clearTarget();
    ui.hasTarget = false;
    ui.trackingActive = false;
    ui.baselineSet = false;
    ui.lastAnomaly = {};
    ui.lastTargetCenter = QPointF();
    ui.lastDeltaX = 0.0;
    ui.lastDeltaY = 0.0;
    ui.lastDisplacement = 0.0;
    ui.lastSpeed = 0.0;
    ui.lastScaleRatio = 1.0;
    appendLog(tr("检测点 %1 已清除锁定目标与运动轨迹").arg(index + 1));
    statusBar()->showMessage(tr("检测点 %1 目标已清除，可重新左键拖拽框选").arg(index + 1), 3000);
    updateFusionDecision();
}

void MainWindow::clearAllTargets()
{
    for (int i = 0; i < kChannelCount; ++i) {
        auto &ui = m_channels[i];
        ui.channel->clearTarget();
        ui.hasTarget = false;
        ui.trackingActive = false;
        ui.baselineSet = false;
        ui.lastAnomaly = {};
        ui.lastTargetCenter = QPointF();
        ui.lastDeltaX = 0.0;
        ui.lastDeltaY = 0.0;
        ui.lastDisplacement = 0.0;
        ui.lastSpeed = 0.0;
        ui.lastScaleRatio = 1.0;
    }
    m_lastReportedSignature.clear();
    appendLog(tr("已清除全部检测点的锁定目标与轨迹"));
    statusBar()->showMessage(tr("已清除全部检测点目标，可在任意一路重新框选"), 3000);
    updateFusionDecision();
}

void MainWindow::broadcastTargetSignature(int sourceIndex, const TargetSignature &sig)
{
    if (!sig.valid || !m_chkCrossViewSync || !m_chkCrossViewSync->isChecked()) return;
    int synced = 0;
    for (int i = 0; i < kChannelCount; ++i) {
        if (i == sourceIndex) continue;
        auto &ui = m_channels[i];
        if (!ui.channel || !ui.channel->isOpen()) continue;
        ui.baselineSet = false;
        ui.hasTarget = false;
        ui.trackingActive = true;
        ui.lastAnomaly = {};
        ui.lastDeltaX = 0.0;
        ui.lastDeltaY = 0.0;
        ui.lastDisplacement = 0.0;
        ui.lastSpeed = 0.0;
        ui.lastScaleRatio = 1.0;
        ui.channel->syncTargetSignature(sig);
        ++synced;
    }
    if (synced > 0) {
        appendLog(tr("已联动同步检测点 %1 的目标特征至其余 %2 路，自动全图搜索并锁定中…")
                      .arg(sourceIndex + 1).arg(synced));
        statusBar()->showMessage(tr("已由检测点 %1 联动其余 %2 路自动搜寻同特征目标")
                                    .arg(sourceIndex + 1).arg(synced), 4000);
        updateFusionDecision();
    }
}

void MainWindow::applyMonitorThresholds()
{
    m_thresholds.tiltThresh = m_spinTiltThresh ? m_spinTiltThresh->value() : 8.0;
    m_thresholds.scaleThresh = m_spinScaleThresh ? (m_spinScaleThresh->value() / 100.0) : 0.15;
    m_thresholds.dispThresh = m_spinDispThresh ? m_spinDispThresh->value() : 15.0;

    for (auto &c : m_channels) {
        c.channel->setThresholds(m_thresholds);
    }
}

void MainWindow::applyDetectorMode(int modeIndex)
{
    for (auto &c : m_channels) {
        c.channel->setDetectorMode(modeIndex);
        c.channel->clearTarget();
        c.baselineSet = false;
        c.hasTarget = false;
        c.trackingActive = false;
        c.lastAnomaly = {};
    }
    updateFusionDecision();
    appendLog(tr("切换检测模式为：%1").arg(m_comboMode->currentText()));
}

void MainWindow::updateFusionDecision()
{
    const qint64 now = monitorNowMs();
    int unavailable = 0, uncalibrated = 0, pending = 0, lost = 0;
    int trackingLive = 0, trackingSearching = 0;
    int tilted = 0, scaledLarger = 0, scaledSmaller = 0, displaced = 0, normalCount = 0;
    QList<int> lostChannels, changedChannels;
    QStringList alertDetails;

    const auto setCard = [](ChannelUi &ui, const QString &badgeText, const QString &badgeColor, const QString &detailText) {
        if (ui.cardBadge) {
            if (ui.cardBadge->text() != badgeText) ui.cardBadge->setText(badgeText);
            const QString s = QStringLiteral(
                "border: none; background-color: %1; color: white; font-weight: bold; font-size: 8.5pt; border-radius: 3px;").arg(badgeColor);
            if (ui.cardBadge->styleSheet() != s) ui.cardBadge->setStyleSheet(s);
        }
        if (ui.cardDetail && ui.cardDetail->text() != detailText)
            ui.cardDetail->setText(detailText);
    };

    for (int i = 0; i < kChannelCount; ++i) {
        auto &ui = m_channels[i];
        const bool fresh = ui.channel->isOpen() && ui.streamAvailable
            && ui.observedAtMs >= 0 && now - ui.observedAtMs >= 0
            && now - ui.observedAtMs <= kMonitorFreshnessMs;
        if (!fresh) {
            ++unavailable;
            const bool opened = ui.channel->isOpen();
            const QString reason = opened ? tr("视频中断/等待新帧") : tr("未连接");
            ui.video->setMonitoringUnavailable(reason);
            setCard(ui, opened ? tr("等待画面") : tr("未连接"), QStringLiteral("#78909c"),
                    opened ? tr("ESP32 无线连接中或画面过期") : tr("点击连接或等待自动发现"));
            continue;
        }
        if (!ui.baselineSet || ui.lastAnomaly.type == AnomalyType::Uncalibrated) {
            ++uncalibrated;
            if (ui.trackingActive && ui.hasTarget) {
                ++trackingLive;
                const QString dxStr = QStringLiteral("%1%2").arg(ui.lastDeltaX >= 0.0 ? QStringLiteral("+") : QString()).arg(ui.lastDeltaX, 0, 'f', 0);
                const QString dyStr = QStringLiteral("%1%2").arg(ui.lastDeltaY >= 0.0 ? QStringLiteral("+") : QString()).arg(ui.lastDeltaY, 0, 'f', 0);
                setCard(ui, tr("跟踪中"), QStringLiteral("#0288d1"),
                        tr("(%1,%2) · Δ%3px (%4,%5) · %6px/s · %7%")
                            .arg(int(std::lround(ui.lastTargetCenter.x())))
                            .arg(int(std::lround(ui.lastTargetCenter.y())))
                            .arg(ui.lastDisplacement, 0, 'f', 1)
                            .arg(dxStr, dyStr)
                            .arg(ui.lastSpeed, 0, 'f', 0)
                            .arg(ui.lastScaleRatio * 100.0, 0, 'f', 0));
            } else if (ui.trackingActive && !ui.hasTarget) {
                ++trackingSearching;
                setCard(ui, tr("搜索中"), QStringLiteral("#f57c00"),
                        tr("目标暂离/遮挡，正在全图重找…（右键清除）"));
            } else {
                setCard(ui, tr("待选目标"), QStringLiteral("#757575"),
                        tr("左键拖拽框选目标即开始实时跟踪（右键清除）"));
            }
            continue;
        }
        const auto &a = ui.lastAnomaly;
        const QString posTag = ui.hasTarget
            ? tr("(%1,%2) ").arg(int(std::lround(ui.lastTargetCenter.x()))).arg(int(std::lround(ui.lastTargetCenter.y())))
            : QString();
        switch (a.type) {
        case AnomalyType::Lost: {
            ++lost; lostChannels << (i + 1);
            setCard(ui, tr("目标丢失"), QStringLiteral("#c62828"), tr("全图搜索中 · 可能移出视野或被遮挡"));
            alertDetails << tr("检测点%1 丢失").arg(i + 1);
            break;
        }
        case AnomalyType::Pending: {
            ++pending;
            setCard(ui, tr("待确认"), QStringLiteral("#8d6e00"),
                    a.description.isEmpty() ? tr("正在持续时间防抖确认…") : a.description);
            break;
        }
        case AnomalyType::Tilted: {
            ++tilted; changedChannels << (i + 1);
            const QString b = tr("倾斜 %1°").arg(a.deltaAngle, 0, 'f', 1);
            setCard(ui, b, QStringLiteral("#e65100"),
                    tr("%1倾斜 %2° · 面积 %3% · Δ%4px")
                        .arg(posTag).arg(a.deltaAngle, 0, 'f', 1).arg(a.scaleRatio * 100.0, 0, 'f', 0).arg(a.displacement, 0, 'f', 1));
            alertDetails << tr("检测点%1 %2").arg(i + 1).arg(b);
            break;
        }
        case AnomalyType::ScaledLarger: {
            ++scaledLarger; changedChannels << (i + 1);
            const double pct = (a.scaleRatio - 1.0) * 100.0;
            const QString b = tr("变大 +%1%").arg(pct, 0, 'f', 0);
            setCard(ui, b, QStringLiteral("#d84315"),
                    tr("%1变大 +%2%（%3%）· Δ%4px")
                        .arg(posTag).arg(pct, 0, 'f', 1).arg(a.scaleRatio * 100.0, 0, 'f', 0).arg(a.displacement, 0, 'f', 1));
            alertDetails << tr("检测点%1 %2").arg(i + 1).arg(b);
            break;
        }
        case AnomalyType::ScaledSmaller: {
            ++scaledSmaller; changedChannels << (i + 1);
            const double pct = (1.0 - a.scaleRatio) * 100.0;
            const QString b = tr("变小 -%1%").arg(pct, 0, 'f', 0);
            setCard(ui, b, QStringLiteral("#d84315"),
                    tr("%1变小 -%2%（%3%）· Δ%4px")
                        .arg(posTag).arg(pct, 0, 'f', 1).arg(a.scaleRatio * 100.0, 0, 'f', 0).arg(a.displacement, 0, 'f', 1));
            alertDetails << tr("检测点%1 %2").arg(i + 1).arg(b);
            break;
        }
        case AnomalyType::Displaced: {
            ++displaced; changedChannels << (i + 1);
            const QString b = tr("移动 %1px").arg(a.displacement, 0, 'f', 0);
            QStringList dirs;
            if (std::abs(a.deltaX) >= 4.0)
                dirs << (a.deltaX > 0 ? tr("右%1").arg(a.deltaX, 0, 'f', 0) : tr("左%1").arg(-a.deltaX, 0, 'f', 0));
            if (std::abs(a.deltaY) >= 4.0)
                dirs << (a.deltaY > 0 ? tr("下%1").arg(a.deltaY, 0, 'f', 0) : tr("上%1").arg(-a.deltaY, 0, 'f', 0));
            const QString dirStr = dirs.isEmpty() ? QString() : tr("（%1）").arg(dirs.join(QStringLiteral("、")));
            setCard(ui, b, QStringLiteral("#8e24aa"),
                    tr("%1Δ%2px%3 · %4px/s · %5%")
                        .arg(posTag).arg(a.displacement, 0, 'f', 1).arg(dirStr).arg(ui.lastSpeed, 0, 'f', 0).arg(a.scaleRatio * 100.0, 0, 'f', 0));
            alertDetails << tr("检测点%1 移动%2px%3").arg(i + 1).arg(a.displacement, 0, 'f', 1).arg(dirStr);
            break;
        }
        case AnomalyType::Normal: {
            ++normalCount;
            const QString metric = a.angleReliable
                ? tr("%1Δ%2px · 面积%3% · 倾角%4°")
                      .arg(posTag).arg(a.displacement, 0, 'f', 1).arg(a.scaleRatio * 100.0, 0, 'f', 0).arg(a.deltaAngle, 0, 'f', 1)
                : tr("%1Δ%2px · 面积%3% · %4px/s")
                      .arg(posTag).arg(a.displacement, 0, 'f', 1).arg(a.scaleRatio * 100.0, 0, 'f', 0).arg(ui.lastSpeed, 0, 'f', 0);
            setCard(ui, tr("正常"), QStringLiteral("#2e7d32"), metric);
            break;
        }
        case AnomalyType::Uncalibrated: break;
        }
    }

    const int changedTotal = tilted + scaledLarger + scaledSmaller + displaced;
    QString state, severity, text, color, snapTag;
    if (lost > 0) {
        state = QStringLiteral("lost");
        if (lost >= 2) {
            severity = QStringLiteral("confirmed");
            color = QStringLiteral("#b71c1c");
            snapTag = QStringLiteral("multi-lost");
            text = tr("🚨 多视角确认告警：目标丢失（%1/3 检测点不可见）").arg(lost);
        } else {
            severity = QStringLiteral("single");
            color = QStringLiteral("#d84315");
            snapTag = QStringLiteral("single-lost");
            text = tr("⚠️ 单视角预警：检测点 %1 目标不可见（另 %2 路正常，疑似遮挡）")
                       .arg(lostChannels.first()).arg(normalCount);
        }
    } else if (changedTotal > 0) {
        state = QStringLiteral("changed");
        QStringList kinds;
        if (scaledLarger > 0) kinds << tr("变大 %1 路").arg(scaledLarger);
        if (scaledSmaller > 0) kinds << tr("变小 %1 路").arg(scaledSmaller);
        if (displaced > 0) kinds << tr("移动 %1 路").arg(displaced);
        if (tilted > 0) kinds << tr("倾斜 %1 路").arg(tilted);
        const QString summary = kinds.join(QStringLiteral(" / "));

        if (changedTotal >= 2) {
            severity = QStringLiteral("confirmed");
            color = QStringLiteral("#c62828");
            snapTag = QStringLiteral("multi-change");
            QString interp;
            if ((scaledLarger > 0 || scaledSmaller > 0) && displaced > 0)
                interp = tr("（目标空间移动）");
            else if (scaledLarger >= 2)
                interp = tr("（目标变大）");
            else if (scaledSmaller >= 2)
                interp = tr("（目标变小）");
            else if (displaced >= 2)
                interp = tr("（目标移动）");
            else if (tilted >= 2)
                interp = tr("（目标倾斜）");
            text = tr("🚨 多视角确认变化：%1%2").arg(summary, interp);
        } else {
            severity = QStringLiteral("single");
            color = QStringLiteral("#ef6c00");
            snapTag = QStringLiteral("single-change");
            text = tr("⚠️ 单视角变化提示：检测点 %1（%2）").arg(changedChannels.first()).arg(summary);
        }
    } else if (unavailable > 0) {
        state = QStringLiteral("unavailable");
        severity = QStringLiteral("incomplete");
        color = QStringLiteral("#6d4c41");
        text = tr("连接未就绪：%1 路 ESP32 无线画面离线").arg(unavailable);
    } else if (uncalibrated > 0) {
        state = QStringLiteral("uncalibrated");
        severity = QStringLiteral("incomplete");
        if (trackingLive > 0 || trackingSearching > 0) {
            color = QStringLiteral("#0277bd");
            const QString searchHint = trackingSearching > 0
                ? tr("，%1 路搜索中").arg(trackingSearching)
                : QString();
            text = tr("🔵 实时跟踪中（%1/3 路已锁定%2）· 点击【锁定基准】可启用异常告警")
                       .arg(trackingLive).arg(searchHint);
        } else {
            color = QStringLiteral("#546e7a");
            text = tr("待选目标：%1 路未锁定（左键拖拽框选目标即开始实时跟踪）").arg(uncalibrated);
        }
    } else if (pending > 0) {
        state = QStringLiteral("pending");
        severity = QStringLiteral("pending");
        color = QStringLiteral("#806500");
        text = tr("变化防抖确认中（%1 路）…").arg(pending);
    } else {
        state = QStringLiteral("normal");
        severity = QStringLiteral("normal");
        color = QStringLiteral("#2e7d32");
        text = tr("✅ 多检测点跟踪稳定：三视角均在基准阈值内");
    }

    if (unavailable > 0 && (lost > 0 || changedTotal > 0))
        text += tr("（%1 路离线）").arg(unavailable);

    m_lblFusionStatus->setProperty("monitorState", state);
    m_lblFusionStatus->setProperty("monitorSeverity", severity);
    if (m_lblFusionStatus->text() != text) m_lblFusionStatus->setText(text);
    const QString style = QStringLiteral(
        "background-color: %1; color: white; font-weight: bold; font-size: 10pt; padding: 4px 8px; border-radius: 5px;").arg(color);
    if (m_lblFusionStatus->styleSheet() != style) m_lblFusionStatus->setStyleSheet(style);
    m_lblFusionDetail->setText(alertDetails.join(QStringLiteral(" | ")));

    const QString signature = QStringLiteral("%1|%2|%3|%4|%5|%6|%7|%8|%9")
        .arg(state, severity).arg(lost).arg(tilted).arg(scaledLarger).arg(scaledSmaller)
        .arg(displaced).arg(unavailable).arg(trackingLive);
    if (signature != m_lastReportedSignature) {
        const QString logEntry = alertDetails.isEmpty() ? text : QStringLiteral("%1 [%2]").arg(text, alertDetails.join(QStringLiteral(", ")));
        appendLog(logEntry);
        m_lastReportedSignature = signature;
    }
}

void MainWindow::exportEventLog()
{
    if (!m_txtEventLog) return;
    const QDir alertDir(QFileInfo(m_settingsPath).dir().filePath(QStringLiteral("alerts")));
    if (!alertDir.mkpath(QStringLiteral("."))) return;
    const QString fileName = QStringLiteral("monitor-log-%1.txt")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd-HHmmss")));
    const QString fullPath = alertDir.filePath(fileName);
    QFile file(fullPath);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        file.write(m_txtEventLog->toPlainText().toUtf8());
        file.close();
        statusBar()->showMessage(tr("事件日志已导出至 %1").arg(fullPath), 5000);
        appendLog(tr("已导出事件日志：%1").arg(fullPath));
    }
}

void MainWindow::appendLog(const QString &text)
{
    if (!m_txtEventLog) return;
    const QString timeStr = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss"));
    m_txtEventLog->appendPlainText(QStringLiteral("[%1] %2").arg(timeStr, text));
}

void MainWindow::toggleChannel(int index)
{
    ChannelUi &ui = m_channels[index];
    if (ui.channel->isOpen()) {
        ui.channel->close();
        ui.connectButton->setText(tr("重连"));
        ui.video->clear();
        ui.video->setStatus(tr("未连接"));
        ui.baselineSet = false;
        ui.hasTarget = false;
        ui.trackingActive = false;
        if (ui.panelBox) ui.panelBox->setTitle(tr("检测点 %1（ESP32 无线）").arg(index + 1));
        updateFusionDecision();
        return;
    }
    if (ui.lastUrl.isValid() && !ui.lastUrl.host().isEmpty()) {
        openChannel(index, ui.lastUrl);
        return;
    }
    if (!m_discovery.isRunning()) {
        m_discovery.start();
        m_discoveryButton->setText(tr("停止查找"));
        m_discoveryStatus->setText(tr("正在查找设备…"));
    }
    statusBar()->showMessage(tr("正在按固定编号查找检测点 #%1（无需关心 COM 口）…").arg(index + 1), 4000);
}

void MainWindow::openChannel(int index, const QUrl &url)
{
    if (index < 0 || index >= kChannelCount) return;
    const QUrl selectedUrl = withVideoProfile(url, m_videoProfile);
    m_discovery.setTarget(index + 1, QHostAddress(selectedUrl.host()));
    ChannelUi &ui = m_channels[index];
    ui.lastUrl = selectedUrl;
    ui.streamAvailable = false;
    ui.observedAtMs = -1;
    ui.baselineSet = false;
    ui.hasTarget = false;
    ui.trackingActive = false;
    ui.lastAnomaly = {};
    ui.lastTargetCenter = QPointF();
    ui.lastDeltaX = 0.0;
    ui.lastDeltaY = 0.0;
    ui.lastDisplacement = 0.0;
    ui.lastSpeed = 0.0;
    ui.lastScaleRatio = 1.0;
    ui.video->clear();
    ui.video->setShowTrajectory(m_chkShowTrajectory ? m_chkShowTrajectory->isChecked() : true);
    ui.automaticConnectionAllowed = true;
    ui.unavailableReason.clear();
    ui.connectButton->setText(tr("断开"));
    ui.channel->setParams(ui.params);
    if (m_spinMinArea) ui.autoParams.minArea = m_spinMinArea->value();
    ui.channel->setAutoParams(ui.autoParams);
    ui.channel->setDetectorMode(m_comboMode ? m_comboMode->currentIndex() : 0);
    ui.channel->setShowMask(m_chkShowMask ? m_chkShowMask->isChecked() : false);
    ui.channel->setImageEnhance(m_chkEnhance ? m_chkEnhance->isChecked() : false);
    ui.channel->setThresholds(m_thresholds);
    ui.channel->open(selectedUrl);
    updateFusionDecision();
}

void MainWindow::setChannelUnavailable(int index, const QString &reason)
{
    if (index < 0 || index >= kChannelCount) return;
    m_discovery.setTarget(index + 1, {});
    ChannelUi &ui = m_channels[index];
    ui.automaticConnectionAllowed = false;
    ui.unavailableReason = reason;
    ui.channel->close();
    ui.video->clear();
    ui.video->setStatus(reason);
    ui.connectButton->setText(tr("重连"));
    ui.deviceAddress.clear();
    ui.baselineSet = false;
    ui.hasTarget = false;
    ui.trackingActive = false;
    updateDiscoveryStatus();
    updateFusionDecision();
}

void MainWindow::updateDiscoveryStatus()
{
    int found = 0;
    for (const auto &channel : m_channels) found += !channel.deviceAddress.isEmpty();
    m_discoveryStatus->setText(tr("已发现 %1/%2 台设备").arg(found).arg(kChannelCount));
}

void MainWindow::onDeviceFound(const DeviceDiscovery::Device &device)
{
    // 严格按 ESP32 固件内固化的 DEVICE_ID (1~3) 映射到固定画面框 1~3，与 USB COM 口无关。
    const int index = device.id - 1;
    if (index < 0 || index >= kChannelCount)
        return;
    ChannelUi &ui = m_channels[index];
    if (!ui.automaticConnectionAllowed) return;
    const QHostAddress currentHost(ui.channel->url().host());
    if (ui.channel->isOpen() && currentHost.isLoopback() && !device.address.isLoopback())
        return;
    const QString address = device.address.toString();

    const qint64 now = monitorNowMs();
    const bool currentlyLive = ui.channel->isOpen() && ui.streamAvailable
        && ui.observedAtMs >= 0 && now - ui.observedAtMs <= kMonitorFreshnessMs;

    if (currentlyLive && ui.channel->url().host() != address) {
        if (!ui.deviceAddress.isEmpty() && ui.deviceAddress != address) {
            m_discoveryStatus->setText(tr("编号冲突：%1 和 %2 都是 #%3，请修改固件 DEVICE_ID")
                                           .arg(ui.deviceAddress, address).arg(device.id));
        }
        return;
    }

    if (!ui.deviceAddress.isEmpty() && ui.deviceAddress != address) {
        // 原地址已离线且同一 ID 分配了新 IP：自动迁移到新地址
        ui.deviceAddress = address;
    } else if (ui.deviceAddress.isEmpty()) {
        ui.deviceAddress = address;
        updateDiscoveryStatus();
        int found = 0;
        for (const auto &ch : m_channels) found += !ch.deviceAddress.isEmpty();
        if (found >= kChannelCount) m_discovery.setInterval(10000);
        appendLog(tr("按固定编号自动绑定机位 #%1（%2）").arg(device.id).arg(address));
    }

    // 保存最近发现的 IP 作为下次单播加速提示（仍经 ID 核对，不锁死 COM 口）
    const QString group = (index == 0 ? QStringLiteral("camera") : QStringLiteral("camera%1").arg(index + 1));
    QSettings saved(m_settingsPath, QSettings::IniFormat);
    if (saved.value(group + QStringLiteral("/lastIp")).toString() != address)
        saved.setValue(group + QStringLiteral("/lastIp"), address);

    QUrl streamUrl = device.bestStreamUrl();
    QUrlQuery query(streamUrl);
    if (!query.hasQueryItem(QStringLiteral("transport")))
        query.addQueryItem(QStringLiteral("transport"), QStringLiteral("simple"));
    if (!device.address.isLoopback()) {
        if (!query.hasQueryItem(QStringLiteral("hq")))
            query.addQueryItem(QStringLiteral("hq"), QStringLiteral("1"));
        query.removeAllQueryItems(QStringLiteral("hqq"));
        query.addQueryItem(QStringLiteral("hqq"), QString::number(m_jpegQualityMode));
    }
    query.removeAllQueryItems(QStringLiteral("id"));
    query.addQueryItem(QStringLiteral("id"), QString::number(device.id));
    streamUrl.setQuery(query);

    if (!ui.channel->isOpen() || (!currentlyLive && ui.channel->url().host() != address))
        openChannel(index, streamUrl);
}

void MainWindow::onStateChanged(int index, MjpegStream::State state, const QString &detail)
{
    if (state != MjpegStream::State::Streaming) {
        m_channels[index].streamAvailable = false;
        m_channels[index].observedAtMs = -1;
        m_channels[index].hasTarget = false;
        if (m_channels[index].panelBox)
            m_channels[index].panelBox->setTitle(tr("检测点 %1（ESP32 无线）").arg(index + 1));
    }
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
        text = tr("接收中：%1").arg(detail);
        break;
    case MjpegStream::State::Reconnecting:
        text = tr("重连中：%1").arg(detail);
        break;
    }
    m_channels[index].video->setStatus(text);
    updateFusionDecision();
}
