# 与界面无关的核心模块，主程序和测试共用。
QT += concurrent
INCLUDEPATH += $$PWD
LIBS += -lopencv_tracking$${OPENCV_VER}$${OPENCV_SUFFIX} -lopencv_video$${OPENCV_VER}$${OPENCV_SUFFIX}

SOURCES += \
    $$PWD/mjpegparser.cpp \
    $$PWD/mjpegstream.cpp \
    $$PWD/udpstream.cpp \
    $$PWD/devicediscovery.cpp \
    $$PWD/colorblobdetector.cpp \
    $$PWD/autocontourdetector.cpp \
    $$PWD/channel.cpp

HEADERS += \
    $$PWD/cameraconfigpath.h \
    $$PWD/mjpegparser.h \
    $$PWD/mjpegstream.h \
    $$PWD/udpstream.h \
    $$PWD/videoprofile.h \
    $$PWD/devicediscovery.h \
    $$PWD/targetdetector.h \
    $$PWD/artifactmonitor.h \
    $$PWD/colorblobdetector.h \
    $$PWD/autocontourdetector.h \
    $$PWD/channel.h
