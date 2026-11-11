#pragma once

#include <QWidget>

#include "channel.h"

// 显示一路画面：等比缩放居中，叠加检测框、画面中心十字和偏移连线。
// 左键点击发出 imageClicked（图像坐标），用于取色。
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

signals:
    void imageClicked(int x, int y);
    // Emitted only when a new image is painted; useful for real viewer acceptance.
    void framePresented(quint64 seq);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

private:
    QRectF imageRect() const;

    QString m_title;
    QString m_status;
    ChannelResult m_result;
    bool m_showMask = false;
    bool m_rotate180 = false;
    qint64 m_lastPaintedImage = 0;
};
