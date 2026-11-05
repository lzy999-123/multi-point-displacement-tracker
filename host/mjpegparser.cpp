#include "mjpegparser.h"

namespace {
const QByteArray kHeaderEnd("\r\n\r\n");
const QByteArray kSoi("\xFF\xD8", 2);
const QByteArray kEoi("\xFF\xD9", 2);

// 在分段头中查找 Content-Length（大小写不敏感），找不到返回 -1。
int contentLength(const QByteArray &headers)
{
    const QList<QByteArray> lines = headers.split('\n');
    for (const QByteArray &raw : lines) {
        const QByteArray line = raw.trimmed();
        const int colon = line.indexOf(':');
        if (colon <= 0)
            continue;
        if (line.left(colon).trimmed().toLower() != "content-length")
            continue;
        bool ok = false;
        const int len = line.mid(colon + 1).trimmed().toInt(&ok);
        return ok && len >= 0 ? len : -1;
    }
    return -1;
}
} // namespace

QList<QByteArray> MjpegParser::feed(const QByteArray &data)
{
    m_buffer.append(data);
    QList<QByteArray> frames;
    QByteArray frame;
    while (parseOne(&frame))
        frames.append(frame);

    if (m_buffer.size() > kMaxBufferBytes) {
        // 长时间找不到完整帧，说明数据流已损坏，清空后从下一个分段重新同步
        m_buffer.clear();
        ++m_dropped;
    }
    return frames;
}

void MjpegParser::reset()
{
    m_buffer.clear();
}

bool MjpegParser::parseOne(QByteArray *frame)
{
    for (;;) {
        // 分段头只在缓冲区开头的一小段里找，避免把 JPEG 数据中的 \r\n\r\n 当成分段头结尾
        const QByteArray head = m_buffer.left(kMaxHeaderBytes);
        const int headerEnd = head.indexOf(kHeaderEnd);
        const int soi = head.indexOf(kSoi);
        if (headerEnd < 0 || (soi >= 0 && soi < headerEnd)) {
            // 分段头还没收全，或数据前面没有分段头：按 JPEG 标记查找
            return parseByMarkers(frame);
        }

        const int len = contentLength(head.left(headerEnd));
        if (len < 0)
            return parseByMarkers(frame);

        const int bodyStart = headerEnd + kHeaderEnd.size();
        if (len == 0 || len > kMaxFrameBytes) {
            // 长度不合理：跳过这个分段头，继续找下一个
            m_buffer.remove(0, bodyStart);
            ++m_dropped;
            continue;
        }
        if (m_buffer.size() < bodyStart + len)
            return false;  // 数据还没收全

        *frame = m_buffer.mid(bodyStart, len);
        m_buffer.remove(0, bodyStart + len);
        if (frame->startsWith(kSoi))
            return true;
        ++m_dropped;  // 长度与内容不符，丢弃后继续
    }
}

bool MjpegParser::parseByMarkers(QByteArray *frame)
{
    const int soi = m_buffer.indexOf(kSoi);
    if (soi < 0) {
        // 保留最后 1 字节，防止 FF D8 被拆在两次数据之间；同时保留可能的分段头
        if (m_buffer.size() > 64 * 1024) {
            m_buffer = m_buffer.right(1);
            ++m_dropped;
        }
        return false;
    }
    const int eoi = m_buffer.indexOf(kEoi, soi + kSoi.size());
    if (eoi < 0)
        return false;

    const int end = eoi + kEoi.size();
    if (end - soi > kMaxFrameBytes) {
        m_buffer.remove(0, end);
        ++m_dropped;
        return false;
    }
    *frame = m_buffer.mid(soi, end - soi);
    m_buffer.remove(0, end);
    return true;
}
