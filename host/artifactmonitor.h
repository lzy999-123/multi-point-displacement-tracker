#pragma once

#include <QString>
#include <QStringList>
#include <QMetaType>
#include <QElapsedTimer>
#include <cmath>
#include <algorithm>

#include <opencv2/core.hpp>

#include "targetdetector.h"

// 藏品异常状态类型
enum class AnomalyType {
    Normal = 0,         // 安全正常
    Tilted,             // 姿态倾斜（倒伏/倾角偏离）
    ScaledLarger,       // 尺寸变大（移近镜头或被换成大件）
    ScaledSmaller,      // 尺寸变小（远离镜头或被换成小件）
    Displaced,          // 位置发生偏移
    Lost,               // 未匹配到目标，不能直接断言被盗
    Uncalibrated,
    Pending
};
Q_DECLARE_METATYPE(AnomalyType)

// 变化量化指标
struct AnomalyMetrics {
    AnomalyType type = AnomalyType::Uncalibrated;
    bool angleReliable = false;
    bool confirming = false;
    double deltaAngle = 0.0;       // 倾角绝对偏差（度，0°～90°）
    double scaleRatio = 1.0;       // 面积变化比率（当前面积 / 基准面积，1.0为无变化）
    double displacement = 0.0;     // 质心像素位移（px）
    double deltaX = 0.0;           // X 方向位移（右正左负，px）
    double deltaY = 0.0;           // Y 方向位移（下正上负，px）
    double heightRatio = 1.0;      // 高度变化比率
    double widthRatio = 1.0;       // 宽度变化比率
    QString description;           // 详细状态描述
    QString badgeText;             // 简短徽章文字（用于视频叠加）
};
Q_DECLARE_METATYPE(AnomalyMetrics)

// 监控告警门限配置
struct MonitorThresholds {
    double tiltThresh = 8.0;       // 倾斜门限（度，默认 8.0°）
    double scaleThresh = 0.15;     // 尺寸变化门限（比率偏差，默认 ±15%）
    double dispThresh = 15.0;      // 质心位移门限（像素，默认 15px）

    bool operator==(const MonitorThresholds &o) const {
        return std::abs(tiltThresh - o.tiltThresh) < 1e-3
            && std::abs(scaleThresh - o.scaleThresh) < 1e-3
            && std::abs(dispThresh - o.dispThresh) < 1e-3;
    }
};
Q_DECLARE_METATYPE(MonitorThresholds)

// 单路相机的基准参考状态
struct BaselineState {
    bool isCalibrated = false;
    Detection baseDetection;
    cv::Mat baseImage;

    void calibrate(const Detection &det, const cv::Mat &frame = cv::Mat()) {
        baseDetection = det;
        if (!frame.empty()) {
            frame.copyTo(baseImage);
        } else {
            baseImage.release();
        }
        isCalibrated = det.valid && (det.area > 0);
    }

    void reset() {
        isCalibrated = false;
        baseDetection = Detection();
        baseImage.release();
    }

    AnomalyMetrics evaluate(const Detection &curr, const MonitorThresholds &th) const {
        AnomalyMetrics m;
        if (!isCalibrated) {
            m.type = AnomalyType::Uncalibrated;
            m.description = QStringLiteral("未设定基准");
            m.badgeText = QStringLiteral("未设基准");
            return m;
        }

        if (!curr.valid || curr.area <= 0) {
            m.type = AnomalyType::Lost;
            m.description = QStringLiteral("未匹配到选定目标，可能移走、遮挡或轮廓提取失败");
            m.badgeText = QStringLiteral("目标不可见");
            return m;
        }

        // 1. 倾角偏差：主轴角度在 [-90, +90] 之间，对称周期为 180°
        double diffAngle = std::abs(curr.angle - baseDetection.angle);
        if (diffAngle > 90.0) {
            diffAngle = std::abs(diffAngle - 180.0);
        }
        m.angleReliable = curr.angleReliable && baseDetection.angleReliable;
        m.deltaAngle = m.angleReliable ? diffAngle : 0.0;

        // 2. 尺度比率与位移计算
        m.scaleRatio = curr.area / std::max(1.0, baseDetection.area);
        m.deltaX = curr.center.x - baseDetection.center.x;
        m.deltaY = curr.center.y - baseDetection.center.y;
        m.displacement = cv::norm(curr.center - baseDetection.center);
        m.heightRatio = static_cast<double>(curr.box.height) / std::max(1, baseDetection.box.height);
        m.widthRatio = static_cast<double>(curr.box.width) / std::max(1, baseDetection.box.width);

        const auto directionHint = [&]() {
            QStringList dirs;
            if (std::abs(m.deltaX) >= 4.0)
                dirs << (m.deltaX > 0 ? QStringLiteral("右移 %1px").arg(m.deltaX, 0, 'f', 0)
                                      : QStringLiteral("左移 %1px").arg(-m.deltaX, 0, 'f', 0));
            if (std::abs(m.deltaY) >= 4.0)
                dirs << (m.deltaY > 0 ? QStringLiteral("下移 %1px").arg(m.deltaY, 0, 'f', 0)
                                      : QStringLiteral("上移 %1px").arg(-m.deltaY, 0, 'f', 0));
            return dirs.isEmpty() ? QString() : QStringLiteral("（%1）").arg(dirs.join(QStringLiteral("、")));
        };

        // 3. 判决优先级：倾斜 > 尺寸变大/变小 > 位移 > 正常
        if (m.angleReliable && m.deltaAngle >= th.tiltThresh) {
            m.type = AnomalyType::Tilted;
            m.badgeText = QStringLiteral("⚠️ 倾斜 %1°").arg(m.deltaAngle, 0, 'f', 1);
            m.description = QStringLiteral("⚠️ 姿态倾斜：%1°（门限 %2°）")
                                .arg(m.deltaAngle, 0, 'f', 1)
                                .arg(th.tiltThresh, 0, 'f', 1);
        } else if (m.scaleRatio >= (1.0 + th.scaleThresh)) {
            m.type = AnomalyType::ScaledLarger;
            const double pct = (m.scaleRatio - 1.0) * 100.0;
            m.badgeText = QStringLiteral("⚠️ 变大 +%1%").arg(pct, 0, 'f', 0);
            m.description = QStringLiteral("⚠️ 目标变大：+%1%（面积比 %2%）")
                                .arg(pct, 0, 'f', 1)
                                .arg(m.scaleRatio * 100.0, 0, 'f', 0);
        } else if (m.scaleRatio <= (1.0 - th.scaleThresh)) {
            m.type = AnomalyType::ScaledSmaller;
            const double pct = (1.0 - m.scaleRatio) * 100.0;
            m.badgeText = QStringLiteral("⚠️ 变小 -%1%").arg(pct, 0, 'f', 0);
            m.description = QStringLiteral("⚠️ 目标变小：-%1%（面积比 %2%）")
                                .arg(pct, 0, 'f', 1)
                                .arg(m.scaleRatio * 100.0, 0, 'f', 0);
        } else if (m.displacement >= th.dispThresh) {
            m.type = AnomalyType::Displaced;
            m.badgeText = QStringLiteral("⚠️ 移动 %1px").arg(m.displacement, 0, 'f', 1);
            m.description = QStringLiteral("⚠️ 位置移动：%1 px%2（门限 %3 px）")
                                .arg(m.displacement, 0, 'f', 1)
                                .arg(directionHint())
                                .arg(th.dispThresh, 0, 'f', 1);
        } else {
            m.type = AnomalyType::Normal;
            m.badgeText = QStringLiteral("图像变化在阈值内");
            m.description = m.angleReliable
                ? QStringLiteral("图像主轴变化 %1°，轮廓面积比 %2%")
                      .arg(m.deltaAngle, 0, 'f', 1).arg(m.scaleRatio * 100.0, 0, 'f', 0)
                : QStringLiteral("轮廓面积与位置在阈值内；当前形状无稳定主轴，不评估倾角");
        }

        return m;
    }
};

// 同一单调时钟供各通道和界面比较新鲜度，不使用各流启动时刻或可校时的墙钟。
inline qint64 monitorNowMs()
{
    QElapsedTimer clock;
    clock.start();
    return clock.msecsSinceReference();
}

constexpr qint64 kMonitorFreshnessMs = 1000;

// 确认依据是持续时间，不能用帧数替代；恢复比触发更谨慎。
class AnomalyDebouncer
{
public:
    void reset() { m_stable = {}; m_candidate = AnomalyType::Uncalibrated; m_since = -1; }
    AnomalyMetrics update(const AnomalyMetrics &raw, qint64 now) {
        if (raw.type == AnomalyType::Uncalibrated) { reset(); return raw; }
        if (raw.type == m_stable.type) {
            m_candidate = raw.type;
            m_since = -1;
            m_stable = raw;
            return raw;
        }
        if (m_candidate != raw.type || m_since < 0 || now < m_since) {
            m_candidate = raw.type;
            m_since = now;
        }
        const qint64 delay = raw.type == AnomalyType::Normal ? 1000 : 500;
        if (now - m_since >= delay) {
            m_stable = raw;
            m_since = -1;
            return raw;
        }
        if (raw.type == AnomalyType::Normal && m_stable.type != AnomalyType::Uncalibrated) {
            auto held = m_stable;
            held.confirming = true;
            held.badgeText += QStringLiteral("（恢复确认中）");
            return held;
        }
        auto pending = raw;
        pending.type = AnomalyType::Pending;
        pending.confirming = true;
        pending.badgeText = QStringLiteral("图像状态待确认");
        pending.description = raw.description;
        return pending;
    }
private:
    AnomalyMetrics m_stable;
    AnomalyType m_candidate = AnomalyType::Uncalibrated;
    qint64 m_since = -1;
};
