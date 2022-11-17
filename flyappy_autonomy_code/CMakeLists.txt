cmake_minimum_required(VERSION 3.16.3)
project(flyappy_autonomy_code)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CMAKE_CXX_EXTENSIONS false)

include(CTest)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(include)

add_library(flyappy src/flyappy.cpp)

add_library(flyappy_ros src/flyappy_ros.cpp)
target_include_directories(flyappy_ros PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(flyappy_ros
  flyappy
  ${catkin_LIBRARIES}
)

add_executable(flyappy_autonomy_code_node src/flyappy_autonomy_code_node.cpp)
target_include_directories(flyappy_autonomy_code_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(flyappy_autonomy_code_node
  flyappy_ros
  ${catkin_LIBRARIES}
)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
