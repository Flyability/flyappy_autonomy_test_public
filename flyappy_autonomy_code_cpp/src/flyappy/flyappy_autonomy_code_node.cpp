#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "flyappy/flyappy_ros.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("flyappy_autonomy_code_cpp");

    flyappy::FlyappyRos flyappy_ros(node);

    rclcpp::spin(node);

    return 0;
}
