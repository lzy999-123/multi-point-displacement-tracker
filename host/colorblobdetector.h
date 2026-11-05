#pragma once

#include "targetdetector.h"

// HSV 颜色阈值参数。OpenCV 的 H 范围是 0～179，S、V 是 0～255。
// hMin > hMax 表示色相跨过 0（例如红色 170～10）。
struct ColorParams {
    int hMin = 100, hMax = 130;  // 默认蓝色
    int sMin = 120, sMax = 255;
    int vMin = 70, vMax = 255;
    int minArea = 150;           // 小于该面积（像素）的连通域视为噪声
    int maxTargets = 1;          // 返回的目标数上限，多目标时调大

    bool operator==(const ColorParams &o) const
    {
        return hMin == o.hMin && hMax == o.hMax && sMin == o.sMin && sMax == o.sMax
               && vMin == o.vMin && vMax == o.vMax && minArea == o.minArea
               && maxTargets == o.maxTargets;
    }
    bool operator!=(const ColorParams &o) const { return !(*this == o); }
};
Q_DECLARE_METATYPE(ColorParams)

// 色块检测：HSV 阈值 → 开闭运算去噪 → 外轮廓 → 按面积从大到小输出质心。
class ColorBlobDetector : public TargetDetector
{
public:
    void setParams(const ColorParams &params) { m_params = params; }
    const ColorParams &params() const { return m_params; }

    QList<Detection> detect(const cv::Mat &bgr) override;
    cv::Mat lastMask() const override { return m_mask; }

    // 以 (x, y) 为中心取 radius 邻域的颜色，生成一组阈值（保留 minArea/maxTargets）。
    // 点在图像外时返回 false。
    static bool pickColor(const cv::Mat &bgr, int x, int y, ColorParams *params, int radius = 4);

    static cv::Mat thresholdMask(const cv::Mat &hsv, const ColorParams &params);

private:
    ColorParams m_params;
    cv::Mat m_hsv, m_mask;
};
