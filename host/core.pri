# 与界面无关的核心模块，主程序和测试共用。
INCLUDEPATH += $$PWD

SOURCES += \
    $$PWD/mjpegparser.cpp \
    $$PWD/mjpegstream.cpp \
    $$PWD/udpstream.cpp \
    $$PWD/devicediscovery.cpp \
    $$PWD/colorblobdetector.cpp \
    $$PWD/channel.cpp

HEADERS += \
    $$PWD/cameraconfigpath.h \
    $$PWD/mjpegparser.h \
    $$PWD/mjpegstream.h \
    $$PWD/udpstream.h \
    $$PWD/devicediscovery.h \
    $$PWD/targetdetector.h \
    $$PWD/colorblobdetector.h \
    $$PWD/channel.h
