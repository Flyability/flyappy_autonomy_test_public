cmake_minimum_required(VERSION 3.16.3)
project(flyappy_main_game)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
