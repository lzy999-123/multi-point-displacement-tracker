QT += core gui widgets network serialport
win32:LIBS += -lwlanapi
CONFIG += c++17 console
CONFIG -= app_bundle
TARGET = live_viewer
OPENCV_MODULES = core imgproc imgcodecs
include(../opencv.pri)
include(../core.pri)
SOURCES += live_viewer.cpp ../mainwindow.cpp ../videowidget.cpp
HEADERS += ../mainwindow.h ../videowidget.h
SOURCES += ../cameraserialsession.cpp ../camerawifisession.cpp
HEADERS += ../cameraserialsession.h ../camerawifisession.h
