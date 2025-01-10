#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

#include "flyappy/flyappy.hpp"

namespace flyappy
{

class FlyappyRos
{
  public:
    FlyappyRos(rclcpp::Node::SharedPtr node);

  private:
    void velocityCallback(const geometry_msgs::msg::Vector3& msg);
    void laserScanCallback(const sensor_msgs::msg::LaserScan& msg);
    void gameEndedCallback(const std_msgs::msg::Bool& msg);

    Flyappy flyappy_;  ///< ROS-free main code

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_acceleration_command_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_velocity_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_game_ended_;
};

}  // namespace flyappy
