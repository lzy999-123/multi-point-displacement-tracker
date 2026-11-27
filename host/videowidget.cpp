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

        // 画面十字参考线
        const QPointF center = target.center();
        p.setPen(QPen(QColor(255, 255, 255, 120), 1, Qt::DashLine));
        p.drawLine(QPointF(center.x() - 15, center.y()), QPointF(center.x() + 15, center.y()));
        p.drawLine(QPointF(center.x(), center.y() - 15), QPointF(center.x(), center.y() + 15));

        // 1. 若已设定基准，绘制基准参考轮廓（青色虚线框 + 基准主轴）
        if (m_monitoringAvailable && m_result.baselineCalibrated && m_result.baselineDetection.valid) {
            const Detection &b = m_result.baselineDetection;
            const QRectF baseRect(map(b.box.x, b.box.y),
                                  map(b.box.x + b.box.width, b.box.y + b.box.height));
            p.setPen(QPen(QColor(0, 220, 255, 180), 2, Qt::DashLine));
            p.setBrush(Qt::NoBrush);
            p.drawRect(baseRect);

            // 基准质心与方向射线
            const QPointF baseCenter = map(b.center.x, b.center.y);
            p.setPen(QPen(QColor(0, 220, 255, 220), 1, Qt::DotLine));
            p.drawEllipse(baseCenter, 4, 4);

            const double bRad = b.angle * kPi / 180.0;
            const double bRayLen = std::max(20.0, b.box.height * 0.45 * sy);
            const QPointF bTip(baseCenter.x() + bRayLen * std::cos(bRad),
                               baseCenter.y() + bRayLen * std::sin(bRad));
            if (b.angleReliable) p.drawLine(baseCenter, bTip);

            // 标注“基准”文字
            p.setPen(QColor(0, 220, 255, 220));
            QFont bFont = p.font();
            bFont.setPointSize(8);
            p.setFont(bFont);
            p.drawText(baseRect.left() + 2, baseRect.top() - 4, QStringLiteral("[基准参考]"));
        }

        // 2. 绘制当前目标与异常形态
        if (m_monitoringAvailable && !m_result.detections.isEmpty()) {
            for (int i = 0; i < m_result.detections.size(); ++i) {
                const Detection &d = m_result.detections[i];
                QColor color;
                if (i == 0) {
                    if (!m_result.baselineCalibrated) {
                        color = QColor(0, 230, 0); // 未设基准时默认绿色
                    } else {
                        switch (m_result.anomaly.type) {
                        case AnomalyType::Normal:
                            color = QColor(0, 230, 0);      // 正常：绿色
                            break;
                        case AnomalyType::Tilted:
                            color = QColor(255, 130, 0);    // 倾斜：橙红色
                            break;
                        case AnomalyType::ScaledLarger:
                        case AnomalyType::ScaledSmaller:
                            color = QColor(255, 215, 0);    // 尺寸异常：亮黄色
                            break;
                        case AnomalyType::Displaced:
                            color = QColor(230, 50, 230);   // 位移：品红色
                            break;
                        case AnomalyType::Lost:
                            color = QColor(255, 50, 50);    // 丢失：红色
                            break;
                        case AnomalyType::Uncalibrated:
                        case AnomalyType::Pending:
                            color = QColor(210, 180, 80);
                            break;
                        }
                    }
                } else {
                    color = QColor(180, 180, 180); // 次要目标浅灰
                }

                // 目标包围盒
                p.setPen(QPen(color, 2));
                p.setBrush(Qt::NoBrush);
                const QRectF dRect(map(d.box.x, d.box.y),
                                   map(d.box.x + d.box.width, d.box.y + d.box.height));
                p.drawRect(dRect);

                // 目标质心
                const QPointF c = map(d.center.x, d.center.y);
                p.setBrush(color);
                p.drawEllipse(c, 4, 4);

                // 主轴倾斜指示线（随着物体倾斜而实时偏转）
                const double rad = d.angle * kPi / 180.0;
                const double rayLen = std::max(25.0, d.box.height * 0.5 * sy);
                const QPointF tip(c.x() + rayLen * std::cos(rad),
                                  c.y() + rayLen * std::sin(rad));
                p.setPen(QPen(color, 2));
                if (d.angleReliable) p.drawLine(c, tip);

                // 箭头指示尖端
                if (d.angleReliable) p.drawEllipse(tip, 2, 2);

                // 若有基准，绘制位移连线
                if (i == 0 && m_result.baselineCalibrated && m_result.baselineDetection.valid) {
                    const QPointF bc = map(m_result.baselineDetection.center.x,
                                           m_result.baselineDetection.center.y);
                    p.setPen(QPen(color, 1, Qt::DashLine));
                    p.drawLine(bc, c);
                }
            }
        } else if (m_monitoringAvailable && m_result.baselineCalibrated) {
            // 已设基准但无当前目标：仅在基准位置绘制轻量红框，不遮挡其余画面
            const Detection &b = m_result.baselineDetection;
            const QRectF baseRect(map(b.box.x, b.box.y),
                                  map(b.box.x + b.box.width, b.box.y + b.box.height));
            p.setPen(QPen(QColor(255, 50, 50, 230), 2, Qt::DashLine));
            p.setBrush(QColor(255, 0, 0, 28));
            p.drawRect(baseRect);
        }

        p.restore(); // 恢复坐标系，选框文字不受旋转影响
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
