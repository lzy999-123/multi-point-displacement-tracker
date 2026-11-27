#include "yolodetector.h"

#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QStringList>
#include <QtConcurrent/QtConcurrent>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#ifdef HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace {
constexpr int kBottleClassId = 39;
constexpr float kDefaultConfThresh = 0.25f;
constexpr float kBottleConfThresh = 0.12f;
constexpr float kPreferredConfThresh = 0.12f;
constexpr float kNmsThresh = 0.40f;

// 居中 Letterbox 几何变换（与 Ultralytics / YOLO26n ONNX 导出一致）
struct LetterboxGeometry {
    cv::Size sourceSize;
    cv::Size resizedSize;
    int inputSize = 640;
    int left = 0;
    int top = 0;
    double scale = 1.0;

    explicit LetterboxGeometry(cv::Size source, int input = 640)
        : sourceSize(source), inputSize(input)
    {
        scale = std::min(double(input) / std::max(1, source.width),
                         double(input) / std::max(1, source.height));
        resizedSize = cv::Size(cvRound(source.width * scale), cvRound(source.height * scale));
        left = (input - resizedSize.width) / 2;
        top = (input - resizedSize.height) / 2;
    }

    cv::Mat prepare(const cv::Mat &source) const
    {
        cv::Mat resized, input;
        if (source.size() == resizedSize) resized = source;
        else cv::resize(source, resized, resizedSize, 0, 0, cv::INTER_LINEAR);
        cv::copyMakeBorder(resized, input, top, inputSize - resizedSize.height - top,
                           left, inputSize - resizedSize.width - left,
                           cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
        return input;
    }

    cv::Rect sourceRect(double x1, double y1, double x2, double y2) const
    {
        const double l = std::clamp((x1 - left) / scale, 0.0, double(sourceSize.width));
        const double t = std::clamp((y1 - top) / scale, 0.0, double(sourceSize.height));
        const double r = std::clamp((x2 - left) / scale, 0.0, double(sourceSize.width));
        const double b = std::clamp((y2 - top) / scale, 0.0, double(sourceSize.height));
        if (r <= l || b <= t) return {};
        return cv::Rect(cvRound(l), cvRound(t), cvRound(r) - cvRound(l), cvRound(b) - cvRound(t));
    }
};

float confThreshFor(int classId, int preferredClassId)
{
    float th = kDefaultConfThresh;
    if (classId == kBottleClassId) th = std::min(th, kBottleConfThresh);
    if (preferredClassId >= 0 && classId == preferredClassId) th = std::min(th, kPreferredConfThresh);
    return th;
}

// 把 ROTATE_90_CLOCKWISE 图像中的检测框映射回原始 WxH 坐标系
cv::Rect unrotate90CwBox(const cv::Rect &rotBox, const cv::Size &origSize)
{
    const int x1 = std::clamp(rotBox.y, 0, origSize.width);
    const int x2 = std::clamp(rotBox.y + rotBox.height, 0, origSize.width);
    const int y1 = std::clamp(origSize.height - (rotBox.x + rotBox.width), 0, origSize.height);
    const int y2 = std::clamp(origSize.height - rotBox.x, 0, origSize.height);
    if (x2 <= x1 || y2 <= y1) return {};
    return cv::Rect(x1, y1, x2 - x1, y2 - y1);
}

QString findModelFile()
{
    QStringList dirs;
    auto addParents = [&](const QString &base) {
        if (base.isEmpty()) return;
        QDir d(base);
        for (int i = 0; i < 5; ++i) {
            const QString p = d.absolutePath();
            if (!dirs.contains(p)) dirs << p;
            if (!d.cdUp()) break;
        }
    };
    if (QCoreApplication::instance())
        addParents(QCoreApplication::applicationDirPath());
    addParents(QDir::currentPath());

    const QStringList rels = {
        QStringLiteral("models/yolo26n.onnx"),
        QStringLiteral("host/models/yolo26n.onnx"),
        QStringLiteral("yolo26n.onnx")
    };
    for (const QString &dir : dirs) {
        for (const QString &rel : rels) {
            const QString cand = QDir(dir).filePath(rel);
            if (QFileInfo::exists(cand)) return cand;
        }
    }
    const QString fallback = QStringLiteral("D:/QT/project/untitled7/yolo26n.onnx");
    if (QFileInfo::exists(fallback)) return fallback;
    return {};
}

static constexpr const char *kCocoEn[] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed",
    "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};

static constexpr const char *kCocoZh[] = {
    "人员", "自行车", "汽车", "摩托车", "飞机", "公交车", "火车", "卡车", "船", "交通灯",
    "消防栓", "停止标志", "停车表", "长椅", "鸟", "猫", "狗", "马", "羊", "牛",
    "大象", "熊", "斑马", "长颈鹿", "背包", "雨伞", "手提包", "领带", "行李箱", "飞盘",
    "滑雪板", "单板", "球", "风筝", "棒球棒", "棒球手套", "滑板", "冲浪板", "网球拍", "瓶子",
    "高脚杯", "杯子", "叉子", "刀", "勺子", "碗", "香蕉", "苹果", "三明治", "橙子",
    "西兰花", "胡萝卜", "热狗", "披萨", "甜甜圈", "蛋糕", "椅子", "沙发", "盆栽", "床",
    "餐桌", "马桶", "电视", "笔记本", "鼠标", "遥控器", "键盘", "手机", "微波炉", "烤箱",
    "烤面包机", "水槽", "冰箱", "书", "时钟", "花瓶", "剪刀", "泰迪熊", "吹风机", "牙刷"
};
} // namespace

struct YoloDetector::Impl {
#ifdef HAS_ONNXRUNTIME
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "DisplacementTrackerYOLO"};
    std::unique_ptr<Ort::Session> session;
    std::vector<std::string> inputNamesStr;
    std::vector<std::string> outputNamesStr;
    std::vector<const char *> inputNames;
    std::vector<const char *> outputNames;
#endif
};

YoloDetector &YoloDetector::instance()
{
    static YoloDetector s_instance;
    return s_instance;
}

YoloDetector::YoloDetector()
    : m_impl(new Impl)
{
}

YoloDetector::~YoloDetector()
{
    delete m_impl;
    m_impl = nullptr;
}

QString YoloDetector::englishClassName(int classId)
{
    if (classId >= 0 && classId < int(std::size(kCocoEn)))
        return QString::fromLatin1(kCocoEn[classId]);
    return QStringLiteral("class_%1").arg(classId);
}

QString YoloDetector::chineseClassName(int classId)
{
    if (classId >= 0 && classId < int(std::size(kCocoZh)))
        return QString::fromUtf8(kCocoZh[classId]);
    return englishClassName(classId);
}

bool YoloDetector::isReady() const
{
    return m_state.load(std::memory_order_acquire) == 2;
}

QString YoloDetector::loadedModelPath() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_modelPath;
}

void YoloDetector::preloadAsync()
{
    if (m_preloadStarted.exchange(true, std::memory_order_acq_rel)) return;
    (void)QtConcurrent::run([this] { ensureLoaded(); });
}

bool YoloDetector::ensureLoaded()
{
    const int st = m_state.load(std::memory_order_acquire);
    if (st == 2) return true;
    if (st == 3) return false;
    std::lock_guard<std::mutex> lock(m_mutex);
    const int st2 = m_state.load(std::memory_order_acquire);
    if (st2 == 2) return true;
    if (st2 == 3) return false;
    m_state.store(1, std::memory_order_release);
    const bool ok = loadModelLocked();
    m_state.store(ok ? 2 : 3, std::memory_order_release);
    return ok;
}

bool YoloDetector::loadModelLocked()
{
#ifndef HAS_ONNXRUNTIME
    return false;
#else
    const QString path = findModelFile();
    if (path.isEmpty()) {
        qInfo() << "[YoloDetector] yolo26n.onnx not found; running in pure visual feature mode.";
        return false;
    }
    try {
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(4);
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        m_impl->session = std::make_unique<Ort::Session>(
            m_impl->env, path.toStdWString().c_str(), opts);

        Ort::AllocatorWithDefaultOptions alloc;
        m_impl->inputNamesStr.clear();
        m_impl->inputNames.clear();
        for (size_t i = 0; i < m_impl->session->GetInputCount(); ++i) {
            auto n = m_impl->session->GetInputNameAllocated(i, alloc);
            m_impl->inputNamesStr.emplace_back(n.get());
        }
        for (const auto &s : m_impl->inputNamesStr) m_impl->inputNames.push_back(s.c_str());

        m_impl->outputNamesStr.clear();
        m_impl->outputNames.clear();
        for (size_t i = 0; i < m_impl->session->GetOutputCount(); ++i) {
            auto n = m_impl->session->GetOutputNameAllocated(i, alloc);
            m_impl->outputNamesStr.emplace_back(n.get());
        }
        for (const auto &s : m_impl->outputNamesStr) m_impl->outputNames.push_back(s.c_str());

        // 执行一次空推理完成内部算子预热
        std::vector<float> dummy(1ULL * 3 * 640 * 640, 0.0f);
        const std::array<int64_t, 4> dims{1, 3, 640, 640};
        auto memInfo = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        Ort::Value inTensor = Ort::Value::CreateTensor<float>(
            memInfo, dummy.data(), dummy.size(), dims.data(), dims.size());
        (void)m_impl->session->Run(Ort::RunOptions{nullptr},
                                   m_impl->inputNames.data(), &inTensor, 1,
                                   m_impl->outputNames.data(), 1);
        m_modelPath = path;
        qInfo() << "[YoloDetector] YOLO26n loaded and warmed up:" << path;
        return true;
    } catch (const std::exception &e) {
        qWarning() << "[YoloDetector] failed to load YOLO26n:" << e.what();
        m_impl->session.reset();
        return false;
    }
#endif
}

std::vector<YoloBox> YoloDetector::runSinglePassLocked(const cv::Mat &bgr, int preferredClassId)
{
    std::vector<YoloBox> results;
#ifndef HAS_ONNXRUNTIME
    Q_UNUSED(bgr);
    Q_UNUSED(preferredClassId);
    return results;
#else
    if (!m_impl->session || bgr.empty() || bgr.type() != CV_8UC3) return results;

    try {
        const LetterboxGeometry geom(bgr.size(), 640);
        const cv::Mat inputImg = geom.prepare(bgr);
        cv::Mat blob;
        cv::dnn::blobFromImage(inputImg, blob, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);

        const std::array<int64_t, 4> dims{1, 3, 640, 640};
        auto memInfo = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        Ort::Value inTensor = Ort::Value::CreateTensor<float>(
            memInfo, reinterpret_cast<float *>(blob.data), blob.total(), dims.data(), dims.size());

        auto outputs = m_impl->session->Run(Ort::RunOptions{nullptr},
                                            m_impl->inputNames.data(), &inTensor, 1,
                                            m_impl->outputNames.data(), 1);
        if (outputs.empty() || !outputs[0].IsTensor()) return results;

        auto info = outputs[0].GetTensorTypeAndShapeInfo();
        auto shape = info.GetShape();
        if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || shape.empty()) return results;

        int rows = 0, cols = 0;
        if (shape.size() == 3 && shape[0] == 1 && shape[1] > 0 && shape[2] > 0) {
            rows = int(shape[1]);
            cols = int(shape[2]);
        } else if (shape.size() == 2 && shape[0] > 0 && shape[1] > 0) {
            rows = int(shape[0]);
            cols = int(shape[1]);
        } else {
            return results;
        }

        float *data = outputs[0].GetTensorMutableData<float>();
        cv::Mat out2d(rows, cols, CV_32F, data);

        std::vector<int> classIds;
        std::vector<float> confs;
        std::vector<cv::Rect> boxes;
        bool nmsApplied = false;

        auto appendBox = [&](float l, float t, float r, float b, float conf, int cls) {
            if (cls < 0 || conf < confThreshFor(cls, preferredClassId)) return;
            if (!std::isfinite(l) || !std::isfinite(t) || !std::isfinite(r)
                || !std::isfinite(b) || !std::isfinite(conf)) return;
            if (std::max({std::abs(l), std::abs(t), std::abs(r), std::abs(b)}) <= 2.0f) {
                l *= 640.0f; t *= 640.0f; r *= 640.0f; b *= 640.0f;
            }
            if (r <= l || b <= t) {
                const float cx = l, cy = t, w = r, h = b;
                if (w <= 1.0f || h <= 1.0f) return;
                l = cx - 0.5f * w; t = cy - 0.5f * h; r = cx + 0.5f * w; b = cy + 0.5f * h;
            }
            const cv::Rect rect = geom.sourceRect(l, t, r, b);
            if (rect.width < 8 || rect.height < 8) return;
            classIds.push_back(cls);
            confs.push_back(conf);
            boxes.push_back(rect);
        };

        if (out2d.cols == 6 || out2d.rows == 6) {
            cv::Mat dets = (out2d.cols == 6) ? out2d : cv::Mat(out2d.t()).clone();
            nmsApplied = true;
            for (int i = 0; i < dets.rows; ++i) {
                const float *row = dets.ptr<float>(i);
                appendBox(row[0], row[1], row[2], row[3], row[4], cvRound(row[5]));
            }
        } else {
            cv::Mat dets = (out2d.rows > out2d.cols) ? out2d : cv::Mat(out2d.t()).clone();
            if (dets.cols < 5) return results;
            const int numCls = dets.cols - 4;
            for (int i = 0; i < dets.rows; ++i) {
                const float *row = dets.ptr<float>(i);
                const float *scores = row + 4;
                int bestCls = 0;
                float bestScore = scores[0];
                for (int c = 1; c < numCls; ++c) {
                    if (scores[c] > bestScore) { bestScore = scores[c]; bestCls = c; }
                }
                if (bestScore >= confThreshFor(bestCls, preferredClassId)) {
                    appendBox(row[0], row[1], row[2], row[3], bestScore, bestCls);
                }
            }
        }

        std::vector<int> keep;
        if (nmsApplied) {
            keep.resize(boxes.size());
            for (size_t i = 0; i < boxes.size(); ++i) keep[i] = int(i);
        } else if (!boxes.empty()) {
            cv::dnn::NMSBoxesBatched(boxes, confs, classIds, 0.0f, kNmsThresh, keep);
        }

        results.reserve(keep.size());
        for (int idx : keep) {
            if (idx < 0 || idx >= int(boxes.size())) continue;
            const cv::Rect clamped = boxes[idx] & cv::Rect(0, 0, bgr.cols, bgr.rows);
            if (clamped.width < 8 || clamped.height < 8) continue;
            results.push_back({clamped, classIds[idx], confs[idx], chineseClassName(classIds[idx])});
        }
    } catch (const std::exception &) {
        results.clear();
    }
    return results;
#endif
}

std::vector<YoloBox> YoloDetector::detect(const cv::Mat &bgr, int preferredClassId,
                                          bool tryRotated90IfEmpty)
{
    if (bgr.empty() || !ensureLoaded()) return {};
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<YoloBox> dets = runSinglePassLocked(bgr, preferredClassId);
    if (!tryRotated90IfEmpty) return dets;

    bool hasPreferred = false;
    for (const auto &d : dets) {
        if (preferredClassId < 0 || d.classId == preferredClassId) {
            hasPreferred = true;
            break;
        }
    }
    if (hasPreferred) return dets;

    cv::Mat rot;
    cv::rotate(bgr, rot, cv::ROTATE_90_CLOCKWISE);
    const auto rotDets = runSinglePassLocked(rot, preferredClassId);
    for (const auto &rd : rotDets) {
        if (preferredClassId >= 0 && rd.classId != preferredClassId) continue;
        const cv::Rect mapped = unrotate90CwBox(rd.box, bgr.size());
        if (mapped.width >= 8 && mapped.height >= 8) {
            dets.push_back({mapped, rd.classId, rd.confidence, rd.className});
        }
    }
    return dets;
}

bool YoloDetector::classifySelection(const cv::Mat &bgr, const cv::Rect &roi, YoloBox &outMatch)
{
    outMatch = YoloBox();
    const cv::Rect sel = roi & cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (bgr.empty() || sel.width < 12 || sel.height < 12 || !ensureLoaded()) return false;

    auto findBest = [&](const std::vector<YoloBox> &cands, YoloBox &best, double &bestScore) {
        const double selArea = std::max(1.0, double(sel.area()));
        for (const auto &c : cands) {
            const cv::Rect inter = c.box & sel;
            const double interArea = std::max(0, inter.area());
            if (interArea <= 0.0) continue;
            const double candArea = std::max(1.0, double(c.box.area()));
            const double selCov = interArea / selArea;
            const double candCov = interArea / candArea;
            const double sizeSim = std::min(selArea, candArea) / std::max(selArea, candArea);
            if (selCov < 0.35 || candCov < 0.22 || sizeSim < 0.18) continue;
            const double score = selCov * 2.4 + candCov * 2.0 + sizeSim * 1.6
                               + std::clamp(double(c.confidence), 0.0, 1.0) * 0.8
                               + (c.classId == kBottleClassId ? 0.35 : 0.0);
            if (score > bestScore) {
                bestScore = score;
                best = c;
            }
        }
    };

    std::lock_guard<std::mutex> lock(m_mutex);
    YoloBox best;
    double bestScore = -1.0;
    const auto upright = runSinglePassLocked(bgr, -1);
    findBest(upright, best, bestScore);

    // 若用户框选的是横躺目标（如已倾倒的瓶子）且正立未命中，再做一次 90° 旋转识别
    if (bestScore < 0.0) {
        cv::Mat rot;
        cv::rotate(bgr, rot, cv::ROTATE_90_CLOCKWISE);
        auto rotCands = runSinglePassLocked(rot, -1);
        for (auto &rc : rotCands) rc.box = unrotate90CwBox(rc.box, bgr.size());
        findBest(rotCands, best, bestScore);
    }

    if (bestScore > 0.0 && best.classId >= 0 && best.box.width >= 12 && best.box.height >= 12) {
        outMatch = best;
        return true;
    }
    return false;
}
