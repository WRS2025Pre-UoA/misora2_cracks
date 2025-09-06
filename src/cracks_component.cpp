#include "misora2_cracks/cracks_component.hpp"

namespace component_cracks
{
EvaluateCracks::EvaluateCracks(const rclcpp::NodeOptions &options)
    : Node("evaluate_cracks", options),
    model(Detection::MODEL_PATH, Detection::ONNX_LOGID, Detection::ONNX_PROVIDER)
{
    receive_image_ = this->create_subscription<MyAdaptedType>("cracks_image",10,std::bind(&EvaluateCracks::update_image_callback,this,std::placeholders::_1));
    
    // crack_size_publisher_ = this->create_publisher<std_msgs::msg::String>("cracks_result_data",10);
    // result_image_publisher_ = this->create_publisher<MyAdaptedType>("cracks_result_image",10);//不要だったらコメントアウト
    publisher_ = this->create_publisher<misora2_custom_msg::msg::Custom>("cracks_results",10);
    // 初期設定-----------------------------------------------------
    if (!std::filesystem::exists(Detection::MODEL_PATH)) {
        std::cerr << "Model file does not exist at path: " << Detection::MODEL_PATH << std::endl;
        throw std::runtime_error("Model file not found.");
    }
    
    colors = Detection::generateRandomColors(model.getNc(), model.getCh());
    names = model.getNames();
    RCLCPP_INFO_STREAM(this->get_logger(),"Complete Initialize");
    // -------------------------------------------------------------------
    // cv::Mat型のreceive_imageを入力としたメーター値検出関数 返り値std::pair<string,cv::Mat>func(cv::Mat ) 画像の出力いらないかも
    // auto [crack_size, result_image] = func(receive_image)
}   

void EvaluateCracks::update_image_callback(const std::unique_ptr<cv::Mat> msg){
    cv::Mat receive_image = std::move(*msg);
    // double crack_width,crack_length; // まだ使用しない
    cv::Mat result_image, trimming_image;
    // RCLCPP_INFO_STREAM(this->get_logger(),"Wait Image");
    // RCLCPP_INFO_STREAM(this->get_logger(), "Received image channels: " << receive_image.channels());
    if (not(receive_image.empty())){
        if (flag == false and receive_image.channels() != 1){// カラー画像である
            // 実装分部
            // 推論実行
            std::vector<YoloResults> objs = model.predict_once(
                receive_image,
                Detection::CONF_THRESHOLD,
                Detection::IOU_THRESHOLD,
                Detection::MASK_THRESHOLD,
                Detection::CONVERSION_CODE
            );
            auto [trimming_image, result_image] = Detection::plot_results(receive_image, objs, colors, names);
            if(trimming_image.channels() == 1) std::cout << "Not found" << std::endl;
            else {
                std::cout << "trimmed: " << trimming_image.size() << std::endl;
                auto [best, lines, corrected] = CracksSize::run_detection(trimming_image);
                if (best.num_lines == 0 || lines.empty()) { // 線を見つけられなかった場合
                    // たった一度の線検出失敗で0と報告していいものだろうか
                    misora2_custom_msg::msg::Custom data;
                    data.result = "0.000,0.000";
                    data.image = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result_image).toImageMsg());
                    publisher_->publish(data);

                    flag = true;
                    RCLCPP_INFO_STREAM(this->get_logger(),"Publish Crack size: "<<  "0.0,0.0");
                }
                else{ // 線を見つけられた場合
                    std::string length = to_string_with_precision(best.total_length,4);
                    std::string width = to_string_with_precision(best.total_width,4);
                    
                    misora2_custom_msg::msg::Custom data;
                    data.result = length + "," + width;
                    data.image = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result_image).toImageMsg());
                    publisher_->publish(data);

                    flag = true;
                    RCLCPP_INFO_STREAM(this->get_logger(),"Publish Crack size: "<<  length + "," + width);
                }
            }
        }
        else if(receive_image.channels() == 1) {
            RCLCPP_INFO_STREAM(this->get_logger(),"Receive: black image" );
            flag = false;// 1 chanelある画像　黒画像 
        } 
    }
}

std::string EvaluateCracks::to_string_with_precision(double value, int precision = 6) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

} //namespace component_cracks
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(component_cracks::EvaluateCracks)