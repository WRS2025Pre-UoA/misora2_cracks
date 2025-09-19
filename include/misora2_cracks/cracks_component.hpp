#ifndef CRACKS_COMPONENT_HPP
#define CRACKS_COMPONENT_HPP

#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <functional>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/type_adapter.hpp>

#include "misora2_cracks/cv_mat_type_adapter.hpp"
#include "misora2_cracks/detection.hpp"
#include "misora2_cracks/size.hpp"
#include "misora2_custom_msg/msg/custom.hpp"

using namespace std::chrono_literals;

namespace component_cracks
{
class EvaluateCracks : public rclcpp::Node
{
public:
    using MyAdaptedType = rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>;

    bool flag = false;

    // クラック検出の設定---------------------
    AutoBackendOnnx model;
    std::vector<cv::Scalar> colors;
    std::unordered_map<int, std::string> names;
    // -------------------------------------
    explicit EvaluateCracks(const rclcpp::NodeOptions &options);
    EvaluateCracks() : EvaluateCracks(rclcpp::NodeOptions{}) {}

private:
    void update_image_callback(const std::unique_ptr<cv::Mat> msg);
    std::string to_string_with_precision(double value, int precision);
    cv::Mat putResult(cv::Mat& image, std::string length, std::string width, std::string area);

    rclcpp::Subscription<MyAdaptedType>::SharedPtr receive_image_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr crack_size_publisher_;
    // rclcpp::Publisher<MyAdaptedType>::SharedPtr result_image_publisher_;
    rclcpp::Publisher<misora2_custom_msg::msg::Custom>::SharedPtr publisher_; // メッセージ型変更
};

} // namespace component_cracks

#endif // CRACKS_COMPONENT_HPP