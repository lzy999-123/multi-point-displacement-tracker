# OpenCV 配置。默认使用 D:/opencv_build/build/install（MSVC 2022 x64 编译的 4.12），
# 可通过环境变量 OPENCV_DIR 覆盖。
OPENCV_DIR = $$(OPENCV_DIR)
isEmpty(OPENCV_DIR): OPENCV_DIR = D:/opencv_build/build/install
OPENCV_VER = 4120

INCLUDEPATH += $$OPENCV_DIR/include
LIBS += -L$$OPENCV_DIR/x64/vc17/lib

CONFIG(debug, debug|release): OPENCV_SUFFIX = d
for(module, OPENCV_MODULES) {
    LIBS += -lopencv_$${module}$${OPENCV_VER}$${OPENCV_SUFFIX}
}
