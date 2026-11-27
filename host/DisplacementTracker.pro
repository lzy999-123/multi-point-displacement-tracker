QT += core gui widgets network serialport concurrent
CONFIG += c++17
TARGET = DisplacementTracker
win32:LIBS += -lwlanapi

OPENCV_MODULES = core imgproc imgcodecs
include(opencv.pri)
include(core.pri)

SOURCES += \
    main.cpp \
    cameraruntime.cpp \
    cameraserialsession.cpp \
    camerawifisession.cpp \
    mainwindow.cpp \
    videowidget.cpp

HEADERS += \
    mainwindow.h \
    cameraruntime.h \
    cameraserialsession.h \
    camerawifisession.h \
    videowidget.h
