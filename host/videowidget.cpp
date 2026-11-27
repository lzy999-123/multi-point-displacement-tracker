#include "videowidget.h"

#include <cmath>
#include <QMouseEvent>
#include <QPainter>

namespace {
constexpr double kPi = 3.14159265358979323846;
} // namespace

VideoWidget::VideoWidget(const QString &title, QWidget *parent)
    : QWidget(parent), m_title(title)
{
    setFixedSize(480, 360);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    setCursor(Qt::CrossCursor);
}

void VideoWidget::setResult(const ChannelResult &result)
{
    m_result = result;
    m_monitoringAvailable = true;
    m_unavailableReason.clear();
    update();
}

void VideoWidget::setStatus(const QString &status)
{
    m_status = status;
    update();
}

void VideoWidget::clear()
{
    m_result = ChannelResult();
    m_monitoringAvailable = false;
    m_lastPaintedImage = 0;
    update();
}

void VideoWidget::setMonitoringUnavailable(const QString &reason)
{
    if (!m_monitoringAvailable && reason == m_unavailableReason) return;
    m_monitoringAvailable = false;
    m_unavailableReason = reason;
    update();
}

QRectF VideoWidget::imageRect() const
{
    const QImage &img = m_result.image;
    if (img.isNull())
        return {};
    QSizeF size = img.size();
    size.scale(this->size(), Qt::KeepAspectRatio);
    return QRectF(QPointF((width() - size.width()) / 2.0, (height() - size.height()) / 2.0), size);
}

void VideoWidget::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::SmoothPixmapTransform, true);
    p.fillRect(rect(), QColor(24, 24, 24));

    const QRectF target = imageRect();
    if (!target.isNull()) {
        p.save();
        if (m_rotate180) {
            p.translate(target.center());
            p.rotate(180);
            p.translate(-target.center());
        }
        const bool useMask = m_showMask && !m_result.mask.isNull();
        p.drawImage(target, useMask ? m_result.mask : m_result.image);

        const double sx = target.width() / m_result.image.width();
        const double sy = target.height() / m_result.image.height();
        auto map = [&](double x, double y) { return QPointF(target.left() + x * sx, target.top() + y * sy); };

        // 画面中心微小十字准星
        const QPointF center = target.center();
        p.setPen(QPen(QColor(255, 255, 255, 95), 1, Qt::DashLine));
        p.drawLine(QPointF(center.x() - 10, center.y()), QPointF(center.x() + 10, center.y()));
        p.drawLine(QPointF(center.x(), center.y() - 10), QPointF(center.x(), center.y() + 10));

        // 1. 短时渐隐运动轨迹尾迹（仅在目标移动时浮现，静止后自动消失）
        if (m_monitoringAvailable && m_showTrajectory && m_result.trajectory.size() >= 2) {
            const int n = m_result.trajectory.size();
            for (int i = 1; i < n; ++i) {
                const double ratio = double(i) / double(n - 1);
                const int alpha = int(30 + 185 * ratio * ratio);
                const double penW = 1.2 + 1.4 * ratio;
                p.setPen(QPen(QColor(0, 230, 255, alpha), penW, Qt::SolidLine, Qt::RoundCap));
                const QPointF a = map(m_result.trajectory[i - 1].x(), m_result.trajectory[i - 1].y());
                const QPointF b = map(m_result.trajectory[i].x(), m_result.trajectory[i].y());
                p.drawLine(a, b);
            }
        }

        // 2. 绘制紧随目标移动的动态跟踪框（无任何固定在原地的参考框）
        QString liveTagText;
        QPointF liveTagAnchor;
        QColor liveTagColor(0, 230, 90);
        bool hasLiveTag = false;
        if (m_monitoringAvailable && !m_result.detections.isEmpty()) {
            for (int i = 0; i < m_result.detections.size(); ++i) {
                const Detection &d = m_result.detections[i];
                QColor color;
                if (i == 0) {
                    if (m_result.baselineCalibrated) {
                        switch (m_result.anomaly.type) {
                        case AnomalyType::Normal:
                            color = (m_result.targetStatus.active && m_result.targetStatus.hasEverMoved)
                                ? QColor(38, 198, 218)       // 正常但曾移动过：青色提示
                                : QColor(0, 235, 95);        // 正常：亮绿色
                            break;
                        case AnomalyType::Tilted:
                            color = m_result.anomaly.toppled
                                ? QColor(255, 61, 0)         // 倾倒：亮红橙色
                                : QColor(255, 145, 0);       // 倾斜：橙色
                            break;
                        case AnomalyType::ScaledLarger:
                        case AnomalyType::ScaledSmaller:
                            color = QColor(255, 215, 0);     // 尺寸异常：亮黄色
                            break;
                        case AnomalyType::Displaced:
                            color = QColor(235, 65, 235);    // 位移：品红色
                            break;
                        case AnomalyType::Lost:
                            color = QColor(255, 65, 65);
                            break;
                        case AnomalyType::Uncalibrated:
                        case AnomalyType::Pending:
                            color = QColor(240, 200, 70);
                            break;
                        }
                    } else if (m_result.targetStatus.active) {
                        const auto &ts = m_result.targetStatus;
                        if (ts.posture == TargetStatus::PostureState::Toppled) {
                            color = QColor(255, 61, 0);      // 倾倒：亮红橙色
                        } else if (ts.posture == TargetStatus::PostureState::Tilted) {
                            color = QColor(255, 145, 0);     // 倾斜：橙色
                        } else if (ts.size == TargetStatus::SizeState::Larger
                                   || ts.size == TargetStatus::SizeState::Smaller) {
                            color = QColor(255, 215, 0);     // 变大/变小：亮黄色
                        } else if (ts.motion == TargetStatus::MotionState::Moving
                                   || ts.motion == TargetStatus::MotionState::Displaced) {
                            color = QColor(235, 65, 235);    // 移动中/已移位：品红色
                        } else if (ts.motion == TargetStatus::MotionState::MovedReturned) {
                            color = QColor(38, 198, 218);    // 曾移动过(已回位)：青色
                        } else {
                            color = QColor(0, 235, 95);      // 静止正常：亮绿色
                        }
                    } else {
                        color = QColor(0, 235, 95);          // 实时跟踪：亮绿色
                    }
                } else {
                    color = QColor(180, 180, 180);
                }

                const QRectF dRect(map(d.box.x, d.box.y),
                                   map(d.box.x + d.box.width, d.box.y + d.box.height));
                // 外框与加粗四角定位标
                p.setPen(QPen(color, 2));
                p.setBrush(Qt::NoBrush);
                p.drawRect(dRect);
                const double corner = std::min(12.0, std::min(dRect.width(), dRect.height()) * 0.28);
                p.setPen(QPen(color, 3));
                p.drawLine(dRect.topLeft(), dRect.topLeft() + QPointF(corner, 0));
                p.drawLine(dRect.topLeft(), dRect.topLeft() + QPointF(0, corner));
                p.drawLine(dRect.topRight(), dRect.topRight() + QPointF(-corner, 0));
                p.drawLine(dRect.topRight(), dRect.topRight() + QPointF(0, corner));
                p.drawLine(dRect.bottomLeft(), dRect.bottomLeft() + QPointF(corner, 0));
                p.drawLine(dRect.bottomLeft(), dRect.bottomLeft() + QPointF(0, -corner));
                p.drawLine(dRect.bottomRight(), dRect.bottomRight() + QPointF(-corner, 0));
                p.drawLine(dRect.bottomRight(), dRect.bottomRight() + QPointF(0, -corner));

                // 目标质心与主轴方向线
                const QPointF c = map(d.center.x, d.center.y);
                p.setBrush(color);
                p.drawEllipse(c, 3.5, 3.5);
                if (d.angleReliable) {
                    const double rad = d.angle * kPi / 180.0;
                    const double rayLen = std::max(20.0, d.box.height * 0.42 * sy);
                    const QPointF tip(c.x() + rayLen * std::cos(rad),
                                      c.y() + rayLen * std::sin(rad));
                    p.setPen(QPen(color, 2));
                    p.drawLine(c, tip);
                    p.drawEllipse(tip, 2.0, 2.0);
                }

                if (i == 0) {
                    liveTagColor = color;
                    if (m_result.targetStatus.active && !m_result.targetStatus.shortTag.isEmpty()) {
                        liveTagText = m_result.targetStatus.shortTag;
                    } else {
                        liveTagText = QStringLiteral("(%1,%2) Δ%3px")
                            .arg(int(std::lround(d.center.x)))
                            .arg(int(std::lround(d.center.y)))
                            .arg(int(std::lround(m_result.displacement)));
                    }
                    // 把标签锚点换算到非旋转窗口坐标系，确保 180° 旋转时文字依然正立
                    const QPointF imgAnchor = m_rotate180
                        ? QPointF(d.box.x + d.box.width, d.box.y + d.box.height)
                        : QPointF(d.box.x, d.box.y);
                    QPointF screenAnchor = map(imgAnchor.x(), imgAnchor.y());
                    if (m_rotate180) {
                        screenAnchor = QPointF(2.0 * target.center().x() - screenAnchor.x(),
                                               2.0 * target.center().y() - screenAnchor.y());
                    }
                    liveTagAnchor = screenAnchor;
                    hasLiveTag = true;
                }
            }
        }

        p.restore(); // 恢复正立坐标系绘制跟随文字或搜索提示

        if (hasLiveTag) {
            QFont tagFont = p.font();
            tagFont.setPointSize(8);
            tagFont.setBold(true);
            p.setFont(tagFont);
            const QFontMetrics fm(tagFont);
            const QSize sz = fm.size(Qt::TextSingleLine, liveTagText) + QSize(8, 4);
            const double tx = qBound(target.left() + 2.0, liveTagAnchor.x(), target.right() - sz.width() - 2.0);
            const double ty = (liveTagAnchor.y() - sz.height() - 3.0 >= target.top() + 2.0)
                                  ? (liveTagAnchor.y() - sz.height() - 3.0)
                                  : (liveTagAnchor.y() + 4.0);
            const QRectF pill(tx, ty, sz.width(), sz.height());
            p.setPen(Qt::NoPen);
            p.setBrush(QColor(15, 20, 28, 175));
            p.drawRoundedRect(pill, 3, 3);
            p.setPen(liveTagColor);
            p.drawText(pill, Qt::AlignCenter, liveTagText);
        } else if (m_monitoringAvailable && (m_result.trackingActive || m_result.baselineCalibrated)) {
            // 目标暂时移出视野或被遮挡：仅在左上角给轻量状态条，不在原地残留空框
            QFont pillFont = p.font();
            pillFont.setPointSize(8);
            pillFont.setBold(true);
            p.setFont(pillFont);
            const QString lostHint = m_result.targetStatus.hasEverMoved
                ? QStringLiteral("目标丢失(曾移动) · 搜索中…")
                : (m_result.baselineCalibrated ? QStringLiteral("目标丢失 · 全图搜索中…")
                                               : QStringLiteral("全图搜索目标中…"));
            const QFontMetrics pillFm(pillFont);
            const QRectF pill(target.left() + 8, target.top() + 8,
                              std::max(156, pillFm.horizontalAdvance(lostHint) + 18), 22);
            p.setPen(Qt::NoPen);
            p.setBrush(QColor(183, 28, 28, 185));
            p.drawRoundedRect(pill, 4, 4);
            p.setPen(Qt::white);
            p.drawText(pill, Qt::AlignCenter, lostHint);
        }
    } else {
        // 无画面时在暗色背景中央给出简洁状态提示，有视频后不在画面上覆盖任何黑框文字
        p.setPen(QColor(185, 190, 198));
        QFont hintFont = p.font();
        hintFont.setPointSize(11);
        p.setFont(hintFont);
        p.drawText(rect(), Qt::AlignCenter,
                   m_status.isEmpty() ? tr("未连接") : m_status);
    }

    // 拖拽框选目标时的实时选框叠加
    if (m_dragging && !target.isNull()) {
        const QRectF dragRect = QRectF(m_dragStart, m_dragCurrent).normalized().intersected(target);
        if (dragRect.width() >= 4.0 && dragRect.height() >= 4.0) {
            p.setPen(QPen(QColor(0, 230, 255), 2, Qt::DashLine));
            p.setBrush(QColor(0, 230, 255, 40));
            p.drawRect(dragRect);
            p.setPen(QColor(0, 240, 255));
            p.drawText(dragRect.topLeft() + QPointF(4, -5), QStringLiteral("松开锁定目标区域"));
        }
    }

    if (!m_result.image.isNull() && m_result.image.cacheKey() != m_lastPaintedImage) {
        m_lastPaintedImage = m_result.image.cacheKey();
        emit framePresented(m_result.seq);
    }
}

QPoint VideoWidget::widgetToImagePoint(const QPointF &pos, const QRectF &target) const
{
    int x = qBound(0, int((pos.x() - target.left()) * m_result.image.width() / target.width()),
                   m_result.image.width() - 1);
    int y = qBound(0, int((pos.y() - target.top()) * m_result.image.height() / target.height()),
                   m_result.image.height() - 1);
    if (m_rotate180) {
        x = m_result.image.width() - 1 - x;
        y = m_result.image.height() - 1 - y;
    }
    return QPoint(x, y);
}

void VideoWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton) {
        m_dragging = false;
        update();
        emit clearRequested();
        return;
    }
    if (event->button() != Qt::LeftButton)
        return;
    const QRectF target = imageRect();
    if (target.isNull() || !target.contains(event->position()))
        return;
    m_dragging = true;
    m_dragStart = event->position();
    m_dragCurrent = event->position();
    update();
}

void VideoWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (!m_dragging) return;
    m_dragCurrent = event->position();
    update();
}

void VideoWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (!m_dragging || event->button() != Qt::LeftButton)
        return;
    m_dragging = false;
    m_dragCurrent = event->position();
    update();

    const QRectF target = imageRect();
    if (target.isNull() || m_result.image.isNull())
        return;

    const QPointF delta = m_dragCurrent - m_dragStart;
    if (std::hypot(delta.x(), delta.y()) >= 12.0) {
        const QPoint p1 = widgetToImagePoint(m_dragStart, target);
        const QPoint p2 = widgetToImagePoint(m_dragCurrent, target);
        const QRect roi(QPoint(qMin(p1.x(), p2.x()), qMin(p1.y(), p2.y())),
                        QPoint(qMax(p1.x(), p2.x()), qMax(p1.y(), p2.y())));
        if (roi.width() >= 12 && roi.height() >= 12) {
            emit roiSelected(roi);
            return;
        }
    }

    if (!target.contains(m_dragStart))
        return;
    const QPoint pt = widgetToImagePoint(m_dragStart, target);
    emit imageClicked(pt.x(), pt.y());
}
