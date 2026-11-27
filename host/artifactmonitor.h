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
    bool toppled = false;          // 是否达到倾倒判定（大幅倾角或纵横比倒伏）
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
    double tiltThresh = 12.0;      // 倾斜门限（度，默认 12.0°，避免轻微视角斜角误报）
    double scaleThresh = 0.15;     // 尺寸变化门限（比率偏差，默认 ±15%）
    double dispThresh = 15.0;      // 质心位移门限（像素，默认 15px）
    double toppleThresh = 35.0;    // 倾倒门限（度，默认 35.0°）

    bool operator==(const MonitorThresholds &o) const {
        return std::abs(tiltThresh - o.tiltThresh) < 1e-3
            && std::abs(scaleThresh - o.scaleThresh) < 1e-3
            && std::abs(dispThresh - o.dispThresh) < 1e-3
            && std::abs(toppleThresh - o.toppleThresh) < 1e-3;
    }
};
Q_DECLARE_METATYPE(MonitorThresholds)

namespace ArtifactGeometry {
// 计算两个 180° 周期主轴角的最小绝对夹角（0°~90°）
inline double principalAngleDiff(double aDeg, double bDeg)
{
    double diff = std::abs(aDeg - bDeg);
    diff = std::fmod(diff, 180.0);
    if (diff > 90.0) diff = 180.0 - diff;
    return diff;
}

// 纵横比倒伏 / 纵深塌陷判定（补充主轴角，覆盖正对镜头倒伏或轮廓短时偏圆的情况）
// 纯粹移近/移远会等比缩放宽高（currAspect / refAspect ≈ 1.0），不会误触发。
inline bool detectToppleByAspect(const Detection &ref, const Detection &curr)
{
    if (!ref.valid || !curr.valid || ref.box.width <= 0 || ref.box.height <= 0
        || curr.box.width <= 0 || curr.box.height <= 0) {
        return false;
    }
    const double refAspect = double(ref.box.height) / double(ref.box.width);
    const double currAspect = double(curr.box.height) / double(curr.box.width);
    const double widthRatio = double(curr.box.width) / double(ref.box.width);
    const double heightRatio = double(curr.box.height) / double(ref.box.height);
    const double aspectCollapse = currAspect / std::max(1e-6, refAspect);

    // 情况 1：竖直目标（如立放瓶子）向左右倒伏变横躺
    const bool sideFlip = (refAspect >= 1.35) && (currAspect <= 0.85)
        && (aspectCollapse <= 0.55) && (heightRatio <= 0.78) && (widthRatio >= 1.15);
    // 情况 2：竖直目标朝向/背向镜头倒下（高度大幅塌陷，宽度基本不变或略增）
    const bool foreshortenedFall = (refAspect >= 1.50)
        && (heightRatio <= 0.58) && (widthRatio >= 0.85) && (aspectCollapse <= 0.62);
    return sideFlip || foreshortenedFall;
}

inline QString formatDirectionCompact(double deltaX, double deltaY)
{
    QStringList dirs;
    if (std::abs(deltaX) >= 4.0)
        dirs << (deltaX > 0 ? QStringLiteral("右%1").arg(deltaX, 0, 'f', 0)
                            : QStringLiteral("左%1").arg(-deltaX, 0, 'f', 0));
    if (std::abs(deltaY) >= 4.0)
        dirs << (deltaY > 0 ? QStringLiteral("下%1").arg(deltaY, 0, 'f', 0)
                            : QStringLiteral("上%1").arg(-deltaY, 0, 'f', 0));
    return dirs.join(QStringLiteral("、"));
}
} // namespace ArtifactGeometry

// 目标跟踪后的实时综合状态（多维度并行输出，互不遮蔽；锁定目标即自动生效）
struct TargetStatus {
    enum class MotionState {
        None = 0,        // 未锁定目标
        Stationary,      // 未移动（从未移动过且当前静止在原位）
        Moving,          // 正在移动
        Displaced,       // 已移动（当前偏离初始位置）
        MovedReturned    // 曾移动过（已回到初始位置附近）
    };
    enum class SizeState {
        Normal = 0,      // 尺寸正常
        Larger,          // 变大
        Smaller          // 变小
    };
    enum class PostureState {
        Normal = 0,      // 未倾倒（姿态正常）
        Tilted,          // 倾斜
        Toppled          // 已倾倒
    };

    bool active = false;            // 是否已建立跟踪参考
    bool visible = false;           // 当前帧目标是否可见
    MotionState motion = MotionState::None;
    SizeState size = SizeState::Normal;
    PostureState posture = PostureState::Normal;

    bool hasEverMoved = false;      // 跟踪以来是否移动过（即使移回原位也保留记录，直到归零/重选）
    bool currentlyMoving = false;   // 当前是否处于移动中
    bool currentlyDisplaced = false;// 当前是否偏离初始位置超过门限
    bool hasEverToppled = false;    // 跟踪以来是否发生过倾倒

    double displacement = 0.0;      // 当前相对跟踪起点的位移（px）
    double maxDisplacement = 0.0;   // 跟踪以来最大位移（px）
    double deltaX = 0.0;            // X 位移（右正左负，px）
    double deltaY = 0.0;            // Y 位移（下正上负，px）
    double speedPxPerSec = 0.0;     // 瞬时平滑速度（px/s）

    double scaleRatio = 1.0;        // 当前面积 / 初始面积
    double sizeChangePct = 0.0;     // 面积变化百分比（正=变大，负=变小）
    double widthRatio = 1.0;        // 当前宽度 / 初始宽度
    double heightRatio = 1.0;       // 当前高度 / 初始高度

    bool angleReliable = false;     // 是否具备可靠主轴倾角
    double deltaAngle = 0.0;        // 主轴倾角偏差（0°~90°）
    bool toppledByAspect = false;   // 是否由纵横比倒伏/高度塌陷触发倾倒判定

    QString motionText;             // 移动维度文字，如 "未移动" / "已移动 35px(右35)" / "曾移动过(已回位)"
    QString sizeText;               // 尺寸维度文字，如 "正常(100%)" / "变大 +26%" / "变小 -22%"
    QString postureText;            // 姿态维度文字，如 "未倾倒" / "倾斜 16°" / "已倾倒(82°)"
    QString summaryText;            // 完整组合状态
    QString shortTag;               // 视频跟踪框旁简短标签
};
Q_DECLARE_METATYPE(TargetStatus)

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

        // 1. 倾角偏差与倒伏判定：主轴角度在 [-90, +90] 之间，对称周期为 180°
        const double diffAngle = ArtifactGeometry::principalAngleDiff(curr.angle, baseDetection.angle);
        m.angleReliable = curr.angleReliable && baseDetection.angleReliable;
        m.deltaAngle = m.angleReliable ? diffAngle : 0.0;

        // 2. 尺度比率与位移计算
        m.scaleRatio = curr.area / std::max(1.0, baseDetection.area);
        m.deltaX = curr.center.x - baseDetection.center.x;
        m.deltaY = curr.center.y - baseDetection.center.y;
        m.displacement = cv::norm(curr.center - baseDetection.center);
        m.heightRatio = static_cast<double>(curr.box.height) / std::max(1, baseDetection.box.height);
        m.widthRatio = static_cast<double>(curr.box.width) / std::max(1, baseDetection.box.width);

        const double effToppleThresh = std::max(th.tiltThresh + 5.0, th.toppleThresh);
        const bool aspectToppled = ArtifactGeometry::detectToppleByAspect(baseDetection, curr);
        m.toppled = (m.angleReliable && m.deltaAngle >= effToppleThresh) || aspectToppled;

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

        // 3. 判决优先级：倾倒/倾斜 > 尺寸变大/变小 > 位移 > 正常
        if (m.toppled) {
            m.type = AnomalyType::Tilted;
            if (m.angleReliable && m.deltaAngle >= th.tiltThresh) {
                m.badgeText = QStringLiteral("⚠️ 倾倒 %1°").arg(m.deltaAngle, 0, 'f', 1);
                m.description = QStringLiteral("⚠️ 目标倾倒：%1°（倾倒门限 %2°）")
                                    .arg(m.deltaAngle, 0, 'f', 1)
                                    .arg(effToppleThresh, 0, 'f', 1);
            } else {
                m.badgeText = QStringLiteral("⚠️ 目标倾倒");
                m.description = QStringLiteral("⚠️ 目标倾倒（形态倒伏：高比 %1%、宽比 %2%）")
                                    .arg(m.heightRatio * 100.0, 0, 'f', 0)
                                    .arg(m.widthRatio * 100.0, 0, 'f', 0);
            }
        } else if (m.angleReliable && m.deltaAngle >= th.tiltThresh) {
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

// 目标状态跟踪器：锁定目标后自动以首帧形态为参考，并行追踪“是否移动过 / 变大变小 / 是否倾倒”。
class TargetStateTracker
{
public:
    explicit TargetStateTracker(qint64 confirmMs = 200) : m_confirmMs(confirmMs) {}

    void setConfirmMs(qint64 confirmMs) { m_confirmMs = std::max<qint64>(0, confirmMs); }
    bool hasReference() const { return m_hasRef; }
    const Detection &reference() const { return m_ref; }

    void reset() {
        m_hasRef = false;
        m_ref = Detection();
        clearHistory();
    }

    void setReference(const Detection &det) {
        if (!det.valid || det.area <= 0.0) {
            reset();
            return;
        }
        m_hasRef = true;
        m_ref = det;
        clearHistory();
    }

    void resetPendingTimers() {
        m_dispSince = -1;
        m_moveSince = -1;
        m_sizeSince = -1;
        m_postureSince = -1;
    }

    TargetStatus update(const Detection &curr, const MonitorThresholds &th, qint64 nowMs,
                        double speedPxPerSec = 0.0, double windowNetDisp = 0.0) {
        TargetStatus st;
        if (!m_hasRef) {
            if (!curr.valid || curr.area <= 0.0) return st;
            setReference(curr);
        }

        st.active = true;
        st.hasEverMoved = m_hasEverMoved;
        st.hasEverToppled = m_hasEverToppled;
        st.maxDisplacement = m_maxDisplacement;

        if (!curr.valid || curr.area <= 0.0) {
            st.visible = false;
            resetPendingTimers();
            st.motion = m_hasEverMoved ? TargetStatus::MotionState::MovedReturned
                                       : TargetStatus::MotionState::Stationary;
            st.size = m_stableSize;
            st.posture = m_stablePosture;
            st.motionText = m_hasEverMoved
                ? QStringLiteral("目标丢失(曾移动最大%1px)").arg(m_maxDisplacement, 0, 'f', 0)
                : QStringLiteral("目标丢失");
            st.sizeText = QStringLiteral("--");
            st.postureText = m_hasEverToppled ? QStringLiteral("曾倾倒") : QStringLiteral("--");
            st.summaryText = m_hasEverMoved
                ? QStringLiteral("目标不可见（曾移动过，最大位移 %1px）").arg(m_maxDisplacement, 0, 'f', 0)
                : QStringLiteral("目标不可见");
            st.shortTag = QStringLiteral("目标丢失");
            return st;
        }

        st.visible = true;
        st.deltaX = curr.center.x - m_ref.center.x;
        st.deltaY = curr.center.y - m_ref.center.y;
        st.displacement = std::hypot(st.deltaX, st.deltaY);
        st.speedPxPerSec = speedPxPerSec;

        st.scaleRatio = curr.area / std::max(1.0, m_ref.area);
        st.sizeChangePct = (st.scaleRatio - 1.0) * 100.0;
        st.widthRatio = double(curr.box.width) / std::max(1, m_ref.box.width);
        st.heightRatio = double(curr.box.height) / std::max(1, m_ref.box.height);

        st.angleReliable = curr.angleReliable && m_ref.angleReliable;
        st.deltaAngle = st.angleReliable ? ArtifactGeometry::principalAngleDiff(curr.angle, m_ref.angle) : 0.0;
        st.toppledByAspect = ArtifactGeometry::detectToppleByAspect(m_ref, curr);

        // 1. 位移与“是否移动过”防抖判定（滞回防抖）
        const bool rawDisplaced = m_stableDisplaced
            ? (st.displacement >= th.dispThresh * 0.75)
            : (st.displacement >= th.dispThresh);
        m_stableDisplaced = debounceBool(rawDisplaced, m_stableDisplaced, m_candDisplaced, m_dispSince, nowMs);
        if (m_stableDisplaced) {
            m_hasEverMoved = true;
            m_maxDisplacement = std::max(m_maxDisplacement, st.displacement);
        }

        // “正在移动”必须基于时间窗净位移 + 速度滞回 + 持续时间防抖，彻底屏蔽静止目标的跟踪框微抖
        const bool rawMoving = m_stableMoving
            ? (speedPxPerSec >= 12.0 && windowNetDisp >= 5.0)
            : (speedPxPerSec >= 24.0 && windowNetDisp >= 8.0);
        const qint64 moveDelay = (m_confirmMs <= 0)
            ? 0
            : (rawMoving ? std::min<qint64>(m_confirmMs, 220) : std::max<qint64>(m_confirmMs, 300));
        m_stableMoving = debounceBoolDelay(rawMoving, m_stableMoving, m_candMoving, m_moveSince, nowMs, moveDelay);

        st.currentlyDisplaced = m_stableDisplaced;
        st.currentlyMoving = m_stableMoving;
        st.hasEverMoved = m_hasEverMoved;
        st.maxDisplacement = m_maxDisplacement;

        if (m_stableMoving) {
            st.motion = TargetStatus::MotionState::Moving;
        } else if (m_stableDisplaced) {
            st.motion = TargetStatus::MotionState::Displaced;
        } else if (m_hasEverMoved) {
            st.motion = TargetStatus::MotionState::MovedReturned;
        } else {
            st.motion = TargetStatus::MotionState::Stationary;
        }

        // 2. 变大 / 变小防抖判定（滞回）
        TargetStatus::SizeState rawSize = TargetStatus::SizeState::Normal;
        if (m_stableSize == TargetStatus::SizeState::Larger) {
            if (st.scaleRatio >= 1.0 + th.scaleThresh * 0.70)
                rawSize = TargetStatus::SizeState::Larger;
            else if (st.scaleRatio <= 1.0 - th.scaleThresh)
                rawSize = TargetStatus::SizeState::Smaller;
        } else if (m_stableSize == TargetStatus::SizeState::Smaller) {
            if (st.scaleRatio <= 1.0 - th.scaleThresh * 0.70)
                rawSize = TargetStatus::SizeState::Smaller;
            else if (st.scaleRatio >= 1.0 + th.scaleThresh)
                rawSize = TargetStatus::SizeState::Larger;
        } else {
            if (st.scaleRatio >= 1.0 + th.scaleThresh)
                rawSize = TargetStatus::SizeState::Larger;
            else if (st.scaleRatio <= 1.0 - th.scaleThresh)
                rawSize = TargetStatus::SizeState::Smaller;
        }
        m_stableSize = debounceEnum(rawSize, m_stableSize, m_candSize, m_sizeSince, nowMs);
        st.size = m_stableSize;

        // 3. 倾斜 / 倾倒防抖判定（主轴倾角 + 纵横比倒伏/塌陷双保险）
        const double effToppleThresh = std::max(th.tiltThresh + 5.0, th.toppleThresh);
        const bool angleToppled = st.angleReliable
            && (m_stablePosture == TargetStatus::PostureState::Toppled
                    ? (st.deltaAngle >= effToppleThresh - 5.0)
                    : (st.deltaAngle >= effToppleThresh));
        const bool angleTilted = st.angleReliable
            && (m_stablePosture == TargetStatus::PostureState::Tilted
                    ? (st.deltaAngle >= th.tiltThresh * 0.75)
                    : (st.deltaAngle >= th.tiltThresh));

        TargetStatus::PostureState rawPosture = TargetStatus::PostureState::Normal;
        if (angleToppled || st.toppledByAspect) {
            rawPosture = TargetStatus::PostureState::Toppled;
        } else if (angleTilted) {
            rawPosture = TargetStatus::PostureState::Tilted;
        }
        m_stablePosture = debounceEnum(rawPosture, m_stablePosture, m_candPosture, m_postureSince, nowMs);
        if (m_stablePosture == TargetStatus::PostureState::Toppled) {
            m_hasEverToppled = true;
        }
        st.posture = m_stablePosture;
        st.hasEverToppled = m_hasEverToppled;

        // 4. 构造直观的中文状态描述与叠加标签
        const QString dir = ArtifactGeometry::formatDirectionCompact(st.deltaX, st.deltaY);
        const QString dirParen = dir.isEmpty() ? QString() : QStringLiteral("(%1)").arg(dir);
        switch (st.motion) {
        case TargetStatus::MotionState::Stationary:
            st.motionText = QStringLiteral("未移动");
            break;
        case TargetStatus::MotionState::Moving:
            st.motionText = QStringLiteral("移动中 Δ%1px%2").arg(st.displacement, 0, 'f', 0).arg(dirParen);
            break;
        case TargetStatus::MotionState::Displaced:
            st.motionText = QStringLiteral("已移动 Δ%1px%2").arg(st.displacement, 0, 'f', 0).arg(dirParen);
            break;
        case TargetStatus::MotionState::MovedReturned:
            st.motionText = QStringLiteral("曾移动过(最大%1px,已回位)").arg(st.maxDisplacement, 0, 'f', 0);
            break;
        case TargetStatus::MotionState::None:
            st.motionText = QStringLiteral("--");
            break;
        }

        switch (st.size) {
        case TargetStatus::SizeState::Larger:
            st.sizeText = QStringLiteral("变大 +%1%").arg(std::max(0.0, st.sizeChangePct), 0, 'f', 0);
            break;
        case TargetStatus::SizeState::Smaller:
            st.sizeText = QStringLiteral("变小 -%1%").arg(std::max(0.0, -st.sizeChangePct), 0, 'f', 0);
            break;
        case TargetStatus::SizeState::Normal:
            st.sizeText = QStringLiteral("尺寸正常(%1%)").arg(st.scaleRatio * 100.0, 0, 'f', 0);
            break;
        }

        switch (st.posture) {
        case TargetStatus::PostureState::Toppled:
            st.postureText = (st.angleReliable && st.deltaAngle >= th.tiltThresh)
                ? QStringLiteral("已倾倒(%1°)").arg(st.deltaAngle, 0, 'f', 0)
                : QStringLiteral("已倾倒(形态倒伏)");
            break;
        case TargetStatus::PostureState::Tilted:
            st.postureText = QStringLiteral("倾斜 %1°").arg(st.deltaAngle, 0, 'f', 1);
            break;
        case TargetStatus::PostureState::Normal:
            st.postureText = QStringLiteral("未倾倒");
            break;
        }

        st.summaryText = QStringLiteral("%1 · %2 · %3").arg(st.motionText, st.sizeText, st.postureText);

        QStringList tags;
        if (st.posture == TargetStatus::PostureState::Toppled) {
            tags << ((st.angleReliable && st.deltaAngle >= th.tiltThresh)
                         ? QStringLiteral("已倾倒 %1°").arg(st.deltaAngle, 0, 'f', 0)
                         : QStringLiteral("已倾倒"));
        } else if (st.posture == TargetStatus::PostureState::Tilted) {
            tags << QStringLiteral("倾斜 %1°").arg(st.deltaAngle, 0, 'f', 0);
        }
        if (st.size == TargetStatus::SizeState::Larger) {
            tags << QStringLiteral("变大 +%1%").arg(std::max(0.0, st.sizeChangePct), 0, 'f', 0);
        } else if (st.size == TargetStatus::SizeState::Smaller) {
            tags << QStringLiteral("变小 -%1%").arg(std::max(0.0, -st.sizeChangePct), 0, 'f', 0);
        }
        if (st.motion == TargetStatus::MotionState::Moving) {
            tags << QStringLiteral("移动中 Δ%1px").arg(st.displacement, 0, 'f', 0);
        } else if (st.motion == TargetStatus::MotionState::Displaced) {
            tags << QStringLiteral("已移动 Δ%1px").arg(st.displacement, 0, 'f', 0);
        } else if (st.motion == TargetStatus::MotionState::MovedReturned) {
            tags << QStringLiteral("曾移动过(已回位)");
        }
        st.shortTag = tags.isEmpty() ? QStringLiteral("未移动 · 正常") : tags.join(QStringLiteral(" · "));
        return st;
    }

private:
    void clearHistory() {
        m_hasEverMoved = false;
        m_hasEverToppled = false;
        m_maxDisplacement = 0.0;
        m_stableDisplaced = false;
        m_candDisplaced = false;
        m_dispSince = -1;
        m_stableMoving = false;
        m_candMoving = false;
        m_moveSince = -1;
        m_stableSize = TargetStatus::SizeState::Normal;
        m_candSize = TargetStatus::SizeState::Normal;
        m_sizeSince = -1;
        m_stablePosture = TargetStatus::PostureState::Normal;
        m_candPosture = TargetStatus::PostureState::Normal;
        m_postureSince = -1;
    }

    bool debounceBoolDelay(bool raw, bool stable, bool &cand, qint64 &since, qint64 nowMs, qint64 delayMs) const {
        if (delayMs <= 0) {
            cand = raw;
            since = -1;
            return raw;
        }
        if (raw == stable) {
            cand = raw;
            since = -1;
            return stable;
        }
        if (cand != raw || since < 0 || nowMs < since) {
            cand = raw;
            since = nowMs;
        }
        if (nowMs - since >= delayMs) {
            since = -1;
            return raw;
        }
        return stable;
    }

    bool debounceBool(bool raw, bool stable, bool &cand, qint64 &since, qint64 nowMs) const {
        return debounceBoolDelay(raw, stable, cand, since, nowMs, m_confirmMs);
    }

    template <typename EnumT>
    EnumT debounceEnum(EnumT raw, EnumT stable, EnumT &cand, qint64 &since, qint64 nowMs) const {
        if (m_confirmMs <= 0) {
            cand = raw;
            since = -1;
            return raw;
        }
        if (raw == stable) {
            cand = raw;
            since = -1;
            return stable;
        }
        if (cand != raw || since < 0 || nowMs < since) {
            cand = raw;
            since = nowMs;
        }
        if (nowMs - since >= m_confirmMs) {
            since = -1;
            return raw;
        }
        return stable;
    }

    qint64 m_confirmMs = 200;
    bool m_hasRef = false;
    Detection m_ref;
    bool m_hasEverMoved = false;
    bool m_hasEverToppled = false;
    double m_maxDisplacement = 0.0;

    bool m_stableDisplaced = false;
    bool m_candDisplaced = false;
    qint64 m_dispSince = -1;

    bool m_stableMoving = false;
    bool m_candMoving = false;
    qint64 m_moveSince = -1;

    TargetStatus::SizeState m_stableSize = TargetStatus::SizeState::Normal;
    TargetStatus::SizeState m_candSize = TargetStatus::SizeState::Normal;
    qint64 m_sizeSince = -1;

    TargetStatus::PostureState m_stablePosture = TargetStatus::PostureState::Normal;
    TargetStatus::PostureState m_candPosture = TargetStatus::PostureState::Normal;
    qint64 m_postureSince = -1;
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
