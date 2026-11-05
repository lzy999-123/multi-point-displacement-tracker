QT += core gui network testlib
CONFIG += c++17 console testcase
CONFIG -= app_bundle
TARGET = core_test

OPENCV_MODULES = core imgproc imgcodecs
include(../opencv.pri)
include(../core.pri)

SOURCES += core_test.cpp
