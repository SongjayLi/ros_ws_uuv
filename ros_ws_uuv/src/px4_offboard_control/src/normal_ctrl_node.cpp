#include "joy_ctrl.h"

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<joy_ctrl>("normal_ctrl_node");
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}