#pragma once

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

// All builds inside this project share its local settings, including Qt Creator.
// A standalone deployed copy keeps using its adjacent configuration file.
inline QString cameraSettingsPath(const QString &executableDirectory = QCoreApplication::applicationDirPath())
{
    QDir directory(executableDirectory);
    for (;;) {
        const QString candidate = directory.filePath("camera-runtime.ini");
        if (QFileInfo::exists(directory.filePath("DisplacementTracker.pro")) && QFileInfo::exists(candidate))
            return candidate;
        if (!directory.cdUp()) break;
    }
    return QDir(executableDirectory).filePath("camera-runtime.ini");
}
