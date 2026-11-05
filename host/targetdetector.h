#pragma once

#include <QList>
#include <QMetaType>

#include <opencv2/core.hpp>

// 一个检测到的目标，坐标单位为输入图像的像素。
struct Detection {
    cv::Rect box;
    cv::Point2d center;  // 目标中心（色块为质心）
    double area = 0.0;   // 像素面积
    double score = 0.0;  // 置信度 0～1；色块检测为面积占比相关的启发值
    int label = 0;       // 类别（色块检测固定为 0，后续多目标/人脸使用）
};

// 检测器接口。阶段 1 只有色块；后续多目标、人脸检测实现同一接口即可接入。
// detect() 在通道的工作线程中调用，同一实例不会被多个线程同时使用。
class TargetDetector
{
public:
    virtual ~TargetDetector() = default;
    // 返回结果按优先级排序，第一个作为跟踪目标。
    virtual QList<Detection> detect(const cv::Mat &bgr) = 0;
    // 最近一次检测的二值掩膜（用于调参显示），不支持时返回空 Mat。
    virtual cv::Mat lastMask() const { return {}; }
};

Q_DECLARE_METATYPE(Detection)
