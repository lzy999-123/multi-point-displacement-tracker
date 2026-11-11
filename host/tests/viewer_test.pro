QT += core gui widgets network testlib
CONFIG += c++17 console testcase
CONFIG -= app_bundle
TARGET = viewer_test
INCLUDEPATH += $$PWD/..
OPENCV_MODULES = core imgproc imgcodecs
include(../opencv.pri)
SOURCES += viewer_test.cpp ../videowidget.cpp
HEADERS += ../videowidget.h ../cameraconfigpath.h
