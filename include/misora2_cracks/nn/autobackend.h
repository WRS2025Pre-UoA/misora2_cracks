#pragma once
#include <filesystem>
#include <vector>
#include <unordered_map>
#include <opencv2/core/mat.hpp>

#include "misora2_cracks/nn/onnx_model_base.h"
#include "misora2_cracks/constants.hpp"

/**
 * @brief Represents the results of YOLO prediction.
 *
 * This structure stores information about a detected object, including its class index,
 * confidence score, bounding box, semantic segmentation mask, and keypoints (if available).
 */
struct YoloResults {
    int class_idx{};                  ///< The class index of the detected object.
    float conf{};                     ///< The confidence score of the detection.
    cv::Rect_<float> bbox;            ///< The bounding box of the detected object.
    cv::Mat mask;                     ///< The semantic segmentation mask (if available).
    std::vector<float> keypoints{};   ///< Keypoints representing the object's pose (if available).
};

struct ImageInfo {
    cv::Size raw_size;  // add additional attrs if you need
};


class AutoBackendOnnx : public OnnxModelBase {
public:
    // constructors
    AutoBackendOnnx(const char* modelPath, const char* logid, const char* provider,
        const std::vector<int>& imgsz, const int& stride,
        const int& nc, std::unordered_map<int, std::string> names);

    AutoBackendOnnx(const char* modelPath, const char* logid, const char* provider);

    // getters
    virtual const std::vector<int>& getImgsz();
    virtual const int& getStride();
    virtual const int& getCh();
    virtual const int& getNc();
    virtual const std::unordered_map<int, std::string>& getNames();
    virtual const std::vector<int64_t>& getInputTensorShape();
    virtual const int& getWidth();
    virtual const int& getHeight();
    virtual const cv::Size& getCvSize();
    virtual const std::string& getTask();

    virtual std::vector<YoloResults> predict_once(cv::Mat& image, const float& conf, const float& iou, const float& mask_threshold, int conversionCode = -1, bool verbose = true);
    std::vector<YoloResults> predict_once(const std::string& imagePath, const float& conf, const float& iou, const  float& mask_threshold, int conversionCode = -1, bool verbose = true);

    virtual void fill_blob(cv::Mat& image, float*& blob, std::vector<int64_t>& inputTensorShape);
    virtual void postprocess_masks(cv::Mat& output0, cv::Mat& output1, ImageInfo para, std::vector<YoloResults>& output,
        int& class_names_num, const float& conf_threshold, const float& iou_threshold,
        int& iw, int& ih, int& mw, int& mh, int& masks_features_num, float mask_threshold = 0.50f);

    virtual void postprocess_detects(cv::Mat& output0, ImageInfo image_info, std::vector<YoloResults>& output,
        int& class_names_num, const float& conf_threshold, const float& iou_threshold);
    virtual void postprocess_kpts(cv::Mat& output0, ImageInfo& image_info, std::vector<YoloResults>& output,
                                  int& class_names_num, const float& conf_threshold, const float& iou_threshold);
    static void _get_mask2(const cv::Mat& mask_info, const cv::Mat& mask_data, const ImageInfo& image_info, cv::Rect bound, cv::Mat& mask_out,
        const float& mask_thresh, int& iw, int& ih, int& mw, int& mh, int& masks_features_num, bool round_downsampled = false);

protected:
    std::vector<int> imgsz_;
    int stride_ = Constants::OnnxInitializers::UNINITIALIZED_STRIDE;
    int nc_ = Constants::OnnxInitializers::UNINITIALIZED_NC; //
    int ch_ = 3;
    std::unordered_map<int, std::string> names_;
    std::vector<int64_t> inputTensorShape_;
    cv::Size cvSize_;
    std::string task_;
    //cv::MatSize cvMatSize_;
};
