#include "videowidget.h"

#include <QMouseEvent>
#include <QPainter>

VideoWidget::VideoWidget(const QString &title, QWidget *parent)
    : QWidget(parent), m_title(title)
{
    setMinimumSize(320, 240);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setCursor(Qt::CrossCursor);
}

void VideoWidget::setResult(const ChannelResult &result)
{
    m_result = result;
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
    p.fillRect(rect(), QColor(24, 24, 24));

    const QRectF target = imageRect();
    if (!target.isNull()) {
        const bool useMask = m_showMask && !m_result.mask.isNull();
        p.setRenderHint(QPainter::SmoothPixmapTransform);
        p.drawImage(target, useMask ? m_result.mask : m_result.image);

        const double sx = target.width() / m_result.image.width();
        const double sy = target.height() / m_result.image.height();
        auto map = [&](double x, double y) { return QPointF(target.left() + x * sx, target.top() + y * sy); };

        // 画面中心十字
        const QPointF center = target.center();
        p.setPen(QPen(QColor(255, 255, 255, 160), 1, Qt::DashLine));
        p.drawLine(QPointF(center.x() - 15, center.y()), QPointF(center.x() + 15, center.y()));
        p.drawLine(QPointF(center.x(), center.y() - 15), QPointF(center.x(), center.y() + 15));

        // 检测框：第一个为跟踪目标（绿色），其余黄色
        for (int i = 0; i < m_result.detections.size(); ++i) {
            const Detection &d = m_result.detections[i];
            const QColor color = i == 0 ? QColor(0, 230, 0) : QColor(255, 210, 0);
            p.setPen(QPen(color, 2));
            p.setBrush(Qt::NoBrush);
            p.drawRect(QRectF(map(d.box.x, d.box.y), map(d.box.x + d.box.width, d.box.y + d.box.height)));
            const QPointF c = map(d.center.x, d.center.y);
            p.setBrush(color);
            p.drawEllipse(c, 3, 3);
            if (i == 0) {
                p.setPen(QPen(color, 1));
                p.drawLine(center, c);
            }
        }
    }

    // 文字信息
    QStringList lines{m_title};
    if (!m_status.isEmpty())
        lines << m_status;
    if (!m_result.image.isNull()) {
        lines << QStringLiteral("%1×%2  %3 fps  检测 %4 ms")
                     .arg(m_result.image.width())
                     .arg(m_result.image.height())
                     .arg(m_result.fps, 0, 'f', 1)
                     .arg(m_result.processMs, 0, 'f', 1);
        lines << (m_result.hasTarget
                      ? QStringLiteral("偏移 x=%1  y=%2").arg(m_result.offsetX).arg(m_result.offsetY)
                      : QStringLiteral("未检测到目标"));
    }
    QFont font = p.font();
    font.setPointSize(10);
    p.setFont(font);
    const QFontMetrics fm(font);
    const int lineH = fm.height();
    QRect box(6, 6, 0, lineH * lines.size() + 8);
    for (const QString &line : lines)
        box.setWidth(qMax(box.width(), fm.horizontalAdvance(line) + 12));
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 0, 0, 150));
    p.drawRect(box);
    p.setPen(Qt::white);
    for (int i = 0; i < lines.size(); ++i)
        p.drawText(box.left() + 6, box.top() + 4 + fm.ascent() + i * lineH, lines[i]);
}

void VideoWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() != Qt::LeftButton)
        return;
    const QRectF target = imageRect();
    if (target.isNull() || !target.contains(event->position()))
        return;
    const double x = (event->position().x() - target.left()) * m_result.image.width() / target.width();
    const double y = (event->position().y() - target.top()) * m_result.image.height() / target.height();
    emit imageClicked(int(x), int(y));
}
