// detection.hpp
#pragma once

#include <random>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <tuple>
#include <unordered_map>
#include <filesystem>
#include "misora2_cracks/nn/onnx_model_base.h"
#include "misora2_cracks/nn/autobackend.h"
#include "misora2_cracks/constants.hpp"

class Detection {
public:
    static constexpr const char* MODEL_PATH = "src/misora2_cracks/checkpoints/best_640_s.onnx";
    static constexpr const char* ONNX_PROVIDER = "cpu";
    static constexpr const char* ONNX_LOGID = "yolov8_inference2";
    static constexpr const float MASK_THRESHOLD = 0.5f;
    static constexpr const float CONF_THRESHOLD = 0.30f;
    static constexpr const float IOU_THRESHOLD = 0.45f;
    static constexpr int CONVERSION_CODE = cv::COLOR_BGR2RGB;

    static const std::vector<std::vector<int>> skeleton;
    static const std::vector<cv::Scalar> posePalette;
    static const std::vector<int> limbColorIndices;
    static const std::vector<int> kptColorIndices;

    static cv::Scalar generateRandomColor(int numChannels);
    static std::vector<cv::Scalar> generateRandomColors(int class_names_num, int numChannels);
    static void plot_masks(cv::Mat img, std::vector<YoloResults>& result, std::vector<cv::Scalar> color,
                           std::unordered_map<int, std::string>& names);
    static void plot_keypoints(cv::Mat& image, const std::vector<YoloResults>& results, const cv::Size& shape);
    static std::tuple<cv::Mat, cv::Mat> plot_results(cv::Mat img, std::vector<YoloResults>& results,
                           std::vector<cv::Scalar> color, std::unordered_map<int, std::string>& names,
                           const cv::Size& shape);
};
