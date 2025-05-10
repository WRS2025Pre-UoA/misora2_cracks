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
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/type_adapter.hpp>

#include "misora2_cracks/cv_mat_type_adapter.hpp"

using namespace std::chrono_literals;

namespace component_cracks
{
class EvaluateCracks : public rclcpp::Node
{
public:
    using MyAdaptedType = rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>;

    bool flag = false;

    explicit EvaluateCracks(const rclcpp::NodeOptions &options);
    EvaluateCracks() : EvaluateCracks(rclcpp::NodeOptions{}) {}

private:
    void update_image_callback(const std::unique_ptr<cv::Mat> msg);
    std::string to_string_with_precision(double value, int precision);

    rclcpp::Subscription<MyAdaptedType>::SharedPtr receive_image_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr crack_size_publisher_;
    rclcpp::Publisher<MyAdaptedType>::SharedPtr result_image_publisher_;
};

} // namespace component_cracks

#endif // CRACKS_COMPONENT_HPP