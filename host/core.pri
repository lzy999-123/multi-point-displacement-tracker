# 与界面无关的核心模块，主程序和测试共用。
QT += concurrent
INCLUDEPATH += $$PWD
LIBS += -lopencv_tracking$${OPENCV_VER}$${OPENCV_SUFFIX} \
        -lopencv_video$${OPENCV_VER}$${OPENCV_SUFFIX} \
        -lopencv_features2d$${OPENCV_VER}$${OPENCV_SUFFIX} \
        -lopencv_dnn$${OPENCV_VER}$${OPENCV_SUFFIX}

# 可选 ONNX Runtime（YOLO26n 语义类别防误锁与完整外接框锚定）
ONNXRUNTIME_DIR = $$(ONNXRUNTIME_DIR)
isEmpty(ONNXRUNTIME_DIR): ONNXRUNTIME_DIR = D:/onnxruntime_gpu/onnxruntime-win-x64-gpu-1.20.1
exists($$ONNXRUNTIME_DIR/include/onnxruntime_cxx_api.h) {
    DEFINES += HAS_ONNXRUNTIME
    INCLUDEPATH += $$ONNXRUNTIME_DIR/include
    LIBS += -L$$ONNXRUNTIME_DIR/lib -lonnxruntime
}

SOURCES += \
    $$PWD/mjpegparser.cpp \
    $$PWD/mjpegstream.cpp \
    $$PWD/udpstream.cpp \
    $$PWD/devicediscovery.cpp \
    $$PWD/colorblobdetector.cpp \
    $$PWD/yolodetector.cpp \
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
    $$PWD/yolodetector.h \
    $$PWD/autocontourdetector.h \
    $$PWD/channel.h
