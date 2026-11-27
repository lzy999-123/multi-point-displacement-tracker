QT += core gui widgets network serialport concurrent
win32:LIBS += -lwlanapi
CONFIG += c++17 console
CONFIG -= app_bundle
TARGET = profile_live
OPENCV_MODULES = core imgproc imgcodecs
include(../opencv.pri)
include(../core.pri)
SOURCES += profile_live.cpp ../mainwindow.cpp ../videowidget.cpp \
           ../cameraruntime.cpp ../cameraserialsession.cpp ../camerawifisession.cpp
HEADERS += ../mainwindow.h ../videowidget.h ../cameraruntime.h \
           ../cameraserialsession.h ../camerawifisession.h
