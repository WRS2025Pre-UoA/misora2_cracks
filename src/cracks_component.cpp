#include "misora2_cracks/cracks_component.hpp"

namespace component_cracks
{
EvaluateCracks::EvaluateCracks(const rclcpp::NodeOptions &options)
    : Node("evaluate_cracks", options)
{
    receive_image_ = this->create_subscription<MyAdaptedType>("cracks_image",10,std::bind(&EvaluateCracks::update_image_callback,this,std::placeholders::_1));
    
    crack_size_publisher_ = this->create_publisher<std_msgs::msg::String>("cracks_result_data",10);
    result_image_publisher_ = this->create_publisher<MyAdaptedType>("cracks_result_image",10);//不要だったらコメントアウト

    // cv::Mat型のreceive_imageを入力としたメーター値検出関数 返り値std::pair<string,cv::Mat>func(cv::Mat ) 画像の出力いらないかも
    // auto [crack_size, result_image] = func(receive_image)
}   

void EvaluateCracks::update_image_callback(const std::unique_ptr<cv::Mat> msg){
    cv::Mat receive_image = std::move(*msg);
    double crack_width,crack_length;
    cv::Mat result_image, trimming_image;

    if (not(receive_image.empty())){
        if (flag == false and receive_image.channels() != 1){// カラー画像である
            // 実装分部
            // auto[result_image, trimming_image] = func1(receive_image); // 成功：検出したテストピースを囲んだ画像、切り抜いた画像　失敗：黒画像ｘ２
            // if( result_image.channels() != 1 and trimming_image.channels() != 1 ){ // 検出成功時
            //     auto[crack_width, crack_length] = func2(trimming_image); // 見つからなかった場合 [0.0,0.0] 線の幅と長さ　double
            //     std_msgs::msg::String msg_S;
            //     std::string text1 = to_string_with_precision(crack_width, 7); // 桁数は後々調整
            //     std::string text2 = to_string_with_precision(crack_length, 7); // 桁数は後々調整
            //     msg_S.data = text1 + "," + text2;
            //     crack_size_publisher->publish(msg_S);
            //     // 取得したメーター値を検出画像に書き込むこと
            //     result_image_publisher_->publish(result_image);
            //     flag = true;
            // }else RCLCPP_INFO_STREAM(this->get_logger(), "Couldn't find meter");
            // テスト用-------------------------------------------
            std_msgs::msg::String msg_S;
            msg_S.data = "1.0,0.04";
            crack_size_publisher_->publish(msg_S);
            result_image_publisher_->publish(receive_image);
            RCLCPP_INFO_STREAM(this->get_logger(),"Publish: "<< receive_image.size() );
            flag = true;
            // ---------------------------------------------------
        }
        else if(receive_image.channels() == 1) flag = false;// 1 chanelある画像　黒画像  
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