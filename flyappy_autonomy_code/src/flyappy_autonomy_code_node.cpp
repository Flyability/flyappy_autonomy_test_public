#include <ros/ros.h>

#include "flyappy_autonomy_code/flyappy_ros.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flyappy_autonomy_code");
    ros::NodeHandle nh;

    FlyappyRos flyappy_ros{nh};

    ros::spin();

    return 0;
}
