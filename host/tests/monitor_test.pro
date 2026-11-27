QT += core gui widgets network testlib
CONFIG += c++17 console testcase
CONFIG -= app_bundle
TARGET = monitor_test
OPENCV_MODULES = core imgproc imgcodecs
include(../opencv.pri)
include(../core.pri)
SOURCES += monitor_test.cpp ../mainwindow.cpp ../videowidget.cpp
HEADERS += ../mainwindow.h ../videowidget.h
