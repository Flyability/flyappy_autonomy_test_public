#include "flyappy/flyappy_ros.hpp"

namespace flyappy
{

inline constexpr uint32_t QUEUE_SIZE = 5u;

FlyappyRos::FlyappyRos(rclcpp::Node::SharedPtr node) : node_(node)
{
    pub_acceleration_command_ = node_->create_publisher<geometry_msgs::msg::Vector3>(
            "/flyappy_acc", QUEUE_SIZE);
    sub_velocity_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
            "/flyappy_vel", QUEUE_SIZE,
            std::bind(&FlyappyRos::velocityCallback, this, std::placeholders::_1));
    sub_laser_scan_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/flyappy_laser_scan", QUEUE_SIZE,
            std::bind(&FlyappyRos::laserScanCallback, this, std::placeholders::_1));
    sub_game_ended_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/flyappy_game_ended", QUEUE_SIZE,
            std::bind(&FlyappyRos::gameEndedCallback, this, std::placeholders::_1));
}

void FlyappyRos::velocityCallback([[maybe_unused]] const geometry_msgs::msg::Vector3& msg)
{
    // Example of publishing acceleration command to Flyappy
    geometry_msgs::msg::Vector3 acc_cmd;
    acc_cmd.x = 0.5;  // move and accelerate Flyappy forward
    acc_cmd.y = 0;
    pub_acceleration_command_->publish(acc_cmd);
}

void FlyappyRos::laserScanCallback(const sensor_msgs::msg::LaserScan& msg)
{
    // Example of printing laser angle and range
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Laser range: %f, angle: %f", msg.ranges[0], msg.angle_min);
}

void FlyappyRos::gameEndedCallback(const std_msgs::msg::Bool& msg)
{
    if (msg.data)
    {
        RCLCPP_INFO(node_->get_logger(), "Crash detected.");
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "End of countdown.");
    }

    flyappy_ = Flyappy{};
}

}  // namespace flyappy
