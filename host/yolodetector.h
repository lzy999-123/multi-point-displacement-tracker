#pragma once

#include <QString>
#include <atomic>
#include <mutex>
#include <vector>

#include <opencv2/core.hpp>

// 单个 YOLO 目标检测结果（图像像素坐标）
struct YoloBox {
    cv::Rect box;
    int classId = -1;
    float confidence = 0.0f;
    QString className;
};

// 线程安全的 YOLO26n ONNX Runtime 单例检测器（借鉴 untitled7 的语义类别锁定与完整外接框锚定）：
// - 框选目标时识别语义类别（如瓶子/杯子/手机等），跨视角联动与丢失重捕只接受同类+同外观目标；
// - 支持 90° 旋转补充检测，解决水平倾倒的瓶子在正立 YOLO 下漏检的问题；
// - 若运行环境未提供模型或未启用 ONNX Runtime，自动静默退化为纯传统多特征跟踪。
class YoloDetector
{
public:
    static YoloDetector &instance();

    // 后台预加载并预热模型（不阻塞界面）
    void preloadAsync();
    // 同步确保模型已完成加载（返回是否可用）
    bool ensureLoaded();
    bool isReady() const;
    QString loadedModelPath() const;

    // 检测画面中的目标；preferredClassId >= 0 时对该类降低置信度门限。
    // 当 tryRotated90IfEmpty 为真且正立方向未检出所需类别时，自动补充 90° 旋转检测（兼容横躺倾倒瓶子）。
    std::vector<YoloBox> detect(const cv::Mat &bgr, int preferredClassId = -1,
                                bool tryRotated90IfEmpty = false);

    // 判定用户框选区域是否属于 YOLO 可稳定识别的语义类别，并返回完整目标外接框
    bool classifySelection(const cv::Mat &bgr, const cv::Rect &roi, YoloBox &outMatch);

    static QString chineseClassName(int classId);
    static QString englishClassName(int classId);

private:
    YoloDetector();
    ~YoloDetector();
    YoloDetector(const YoloDetector &) = delete;
    YoloDetector &operator=(const YoloDetector &) = delete;

    std::vector<YoloBox> runSinglePassLocked(const cv::Mat &bgr, int preferredClassId);
    bool loadModelLocked();

    mutable std::mutex m_mutex;
    std::atomic<int> m_state{0}; // 0=unloaded, 1=loading, 2=ready, 3=failed
    std::atomic<bool> m_preloadStarted{false};
    QString m_modelPath;

    struct Impl;
    Impl *m_impl = nullptr;
};
