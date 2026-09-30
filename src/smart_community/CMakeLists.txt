cmake_minimum_required(VERSION 3.16)
project(smart_community)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

# 安装文件夹
install(DIRECTORY
  launch
  worlds
  models
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/traffic_timer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

