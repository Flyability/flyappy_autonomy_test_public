cmake_minimum_required(VERSION 3.28)
project(flyappy_autonomy_code)

option(BUILD_WITH_ROS "Build ROS components" ON)

include(CTest)


find_package(ament_cmake REQUIRED)
find_package(rclcpp CONFIG REQUIRED)

find_package(std_msgs CONFIG REQUIRED)
find_package(geometry_msgs CONFIG REQUIRED)
find_package(sensor_msgs CONFIG REQUIRED)


add_library(flyappy_interface INTERFACE)
target_include_directories(flyappy_interface INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_library(flyappy
    src/flyappy/flyappy.cpp
)
target_link_libraries(flyappy PUBLIC
    flyappy_interface
)

if(BUILD_WITH_ROS)
    add_library(flyappy_ros
        src/flyappy/flyappy_ros.cpp
    )
    target_link_libraries(flyappy_ros PUBLIC
        flyappy
    )
    ament_target_dependencies(flyappy_ros PUBLIC
        rclcpp
        geometry_msgs
        sensor_msgs
        std_msgs
    )

    add_executable(flyappy_autonomy_code_node src/flyappy/flyappy_autonomy_code_node.cpp)
    target_link_libraries(flyappy_autonomy_code_node PRIVATE flyappy_ros)
endif()


if(BUILD_TESTING)
    find_package(GTest CONFIG REQUIRED)
    add_subdirectory(tests)
endif()
