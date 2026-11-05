#pragma once

#include <QByteArray>
#include <QList>

// 把 HTTP multipart/x-mixed-replace 字节流切成一张张 JPEG。
// 优先使用每个分段的 Content-Length；没有该头时按 JPEG 的 SOI(FFD8)/EOI(FFD9) 标记查找。
// 与网络无关，可直接用字节数据做单元测试。
class MjpegParser
{
public:
    static constexpr int kMaxFrameBytes = 2 * 1024 * 1024;
    static constexpr int kMaxBufferBytes = 4 * 1024 * 1024;
    static constexpr int kMaxHeaderBytes = 1024;

    // 追加收到的数据，返回本次新解析出的完整 JPEG（按到达顺序）。
    QList<QByteArray> feed(const QByteArray &data);
    void reset();

    // 因格式错误或超长被丢弃的数据次数，用于诊断。
    int droppedCount() const { return m_dropped; }

private:
    bool parseOne(QByteArray *frame);
    bool parseByMarkers(QByteArray *frame);

    QByteArray m_buffer;
    int m_dropped = 0;
};
