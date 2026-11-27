#pragma once

#include <QList>
#include <QMetaType>

#include <opencv2/core.hpp>

// 一个检测到的目标，坐标单位为输入图像的像素。
struct Detection {
    cv::Rect box;               // 水平正外接矩形
    cv::RotatedRect rotBox;     // 最小外接旋转矩形
    cv::Point2d center;         // 目标中心（质心）
    double area = 0.0;          // 像素面积
    double angle = 0.0;         // 图像主轴角度，180°周期；仅 angleReliable 时可用于比较
    bool angleReliable = false; // 近圆/近方轮廓没有稳定主轴
    double aspectRatio = 0.0;   // 宽高比 (height / max(1, width))
    double score = 0.0;         // 置信度 / 显著度得分 0～1
    int label = 0;              // 类别标签
    bool valid = false;         // 是否为有效目标
};

// 检测器接口。支持免标定轮廓提取、色块检测或未来模型接入。
// detect() 在通道的工作线程中调用，同一实例不会被多个线程同时使用。
class TargetDetector
{
public:
    virtual ~TargetDetector() = default;
    // 返回结果按优先级排序，第一个作为主目标。
    virtual QList<Detection> detect(const cv::Mat &bgr) = 0;
    // 最近一次检测的二值掩膜（用于调参或界面展示），不支持时返回空 Mat。
    virtual cv::Mat lastMask() const { return {}; }
};

Q_DECLARE_METATYPE(Detection)
