#include "rclcpp/rclcpp.hpp"
#include "rm_infantry/auto_aim_node.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rm_infantry::AutoAimNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
