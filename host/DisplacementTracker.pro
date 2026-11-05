QT += core gui widgets network
CONFIG += c++17
TARGET = DisplacementTracker

OPENCV_MODULES = core imgproc imgcodecs
include(opencv.pri)
include(core.pri)

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    videowidget.cpp

HEADERS += \
    mainwindow.h \
    videowidget.h
