#pragma once

#include <QWidget>

#include "channel.h"

// 显示一路画面：等比缩放居中，叠加检测框、画面中心十字和偏移连线。
// 左键拖拽发出 roiSelected（图像坐标 ROI 框选），单击发出 imageClicked（图像坐标）。
class VideoWidget : public QWidget
{
    Q_OBJECT
public:
    explicit VideoWidget(const QString &title, QWidget *parent = nullptr);

    void setResult(const ChannelResult &result);
    void setStatus(const QString &status);
    void setShowMask(bool show) { m_showMask = show; update(); }
    void setRotate180(bool rotate) { m_rotate180 = rotate; update(); }
    bool rotate180() const { return m_rotate180; }
    void clear();
    void setMonitoringUnavailable(const QString &reason);

signals:
    void imageClicked(int x, int y);
    void roiSelected(const QRect &roi);
    // Emitted only when a new image is painted; useful for real viewer acceptance.
    void framePresented(quint64 seq);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    QRectF imageRect() const;
    QPoint widgetToImagePoint(const QPointF &pos, const QRectF &target) const;

    QString m_title;
    QString m_status;
    ChannelResult m_result;
    bool m_showMask = false;
    bool m_rotate180 = false;
    bool m_dragging = false;
    QPointF m_dragStart;
    QPointF m_dragCurrent;
    qint64 m_lastPaintedImage = 0;
    bool m_monitoringAvailable = false;
    QString m_unavailableReason;
};
