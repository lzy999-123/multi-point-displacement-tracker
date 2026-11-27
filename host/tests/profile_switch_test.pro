QT += core gui widgets network serialport concurrent testlib
win32:LIBS += -lwlanapi
CONFIG += c++17 console testcase
CONFIG -= app_bundle
TARGET = profile_switch_test
OPENCV_MODULES = core imgproc imgcodecs
include(../opencv.pri)
include(../core.pri)
SOURCES += profile_switch_test.cpp ../mainwindow.cpp ../videowidget.cpp \
           ../cameraruntime.cpp ../cameraserialsession.cpp ../camerawifisession.cpp
HEADERS += ../mainwindow.h ../videowidget.h ../cameraruntime.h \
           ../cameraserialsession.h ../camerawifisession.h
