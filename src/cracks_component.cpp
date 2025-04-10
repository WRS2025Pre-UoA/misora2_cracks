#include "misora2_cracks/cracks_component.hpp"

namespace component_cracks
{
EvaluateCracks::EvaluateCracks(const rclcpp::NodeOptions &options)
    : Node("evaluate_cracks", options)
{
    receive_image_ = this->create_subscription<MyAdaptedType>("cracks_image",10,std::bind(&EvaluateCracks::update_image_callback,this,std::placeholders::_1));
    
    crack_size_publisher_ = this->create_publisher<std_msgs::msg::String>("cracks_result_data",10);
    result_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("cracks_result_image",10);//不要だったらコメントアウト

    // cv::Mat型のreceive_imageを入力としたメーター値検出関数 返り値std::pair<string,cv::Mat>func(cv::Mat ) 画像の出力いらないかも
    // auto [crack_size, result_image] = func(receive_image)
}   

void EvaluateCracks::update_image_callback(const std::unique_ptr<cv::Mat> msg){
    receive_image = std::move(*msg);

    RCLCPP_INFO_STREAM(this->get_logger(),"Receive image address: " << &(msg->data));
    
}

} //namespace component_cracks
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(component_cracks::EvaluateCracks)