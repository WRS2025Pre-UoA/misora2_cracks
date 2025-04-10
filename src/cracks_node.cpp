#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "misora2_cracks/cracks_component.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    auto node = std::make_shared<component_cracks::EvaluateCracks>();
    exe.add_node(node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
}