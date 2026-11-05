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
    void clear();

signals:
    void imageClicked(int x, int y);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

private:
    QRectF imageRect() const;

    QString m_title;
    QString m_status;
    ChannelResult m_result;
    bool m_showMask = false;
};
