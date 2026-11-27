#pragma once

#include <QUrl>
#include <QUrlQuery>
#include "../esp32cam/displacement_cam/udp_video_profile.h"

inline QString videoProfileKey(int profile)
{
    if (profile == VideoProfile::Baseline10) return QStringLiteral("baseline10");
    if (profile == VideoProfile::Svga) return QStringLiteral("svga");
    return profile == VideoProfile::Qvga25 ? QStringLiteral("qvga25") : QStringLiteral("vga12");
}

inline int videoProfileFromUrl(const QUrl &url)
{
    const QString value = QUrlQuery(url).queryItemValue(QStringLiteral("profile"));
    if (value == QStringLiteral("baseline10")) return VideoProfile::Baseline10;
    if (value == QStringLiteral("vga12")) return VideoProfile::Vga12;
    if (value == QStringLiteral("qvga25")) return VideoProfile::Qvga25;
    if (value == QStringLiteral("svga")) return VideoProfile::Svga;
    return -1;
}

inline QString videoProfileCadence(int profile)
{
    const auto preset = VideoProfile::preset(unsigned(profile));
    return preset.fps ? QStringLiteral("%1 fps").arg(preset.fps) : QStringLiteral("自适应");
}

inline QUrl withVideoProfile(QUrl url, int profile)
{
    if (url.scheme() != QStringLiteral("udp")) {
        url.setScheme(QStringLiteral("udp"));
        url.setPort(5006);
        url.setPath({});
    }
    QUrlQuery query(url);
    for (const QString &key : {QStringLiteral("profile"), QStringLiteral("fps"), QStringLiteral("qmin"),
                              QStringLiteral("qmax"), QStringLiteral("bytes"), QStringLiteral("tos")})
        query.removeAllQueryItems(key);
    query.addQueryItem(QStringLiteral("profile"), videoProfileKey(profile));
    url.setQuery(query);
    return url;
}
