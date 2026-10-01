cmake_minimum_required(VERSION 3.16)
project(my_robot)

find_package(ament_cmake REQUIRED)

# 机器人描述 / 启动文件 / RViz 配置 / Cartographer 配置 / 自检工具 / 实拍图
install(DIRECTORY
  urdf
  launch
  rviz
  config
  maps
  tools
  doc
  DESTINATION share/${PROJECT_NAME}/
)

# 可执行脚本(自动跑遍全场建图用的航点跟踪节点)
install(PROGRAMS
  scripts/survey_driver.py
  scripts/navigate_traffic_lights.py
  scripts/amcl_tf_bridge.py
  scripts/map_obstacles_cloud.py
  scripts/traffic_light_vision.py
  DESTINATION lib/${PROJECT_NAME}
)

# 说明: 这里刻意不在 CMake 配置阶段跑 xacro ——
# my_robot.urdf.xacro 里用的是 $(find my_robot)/urdf/sensors.xacro,
# 而配置阶段包还没进 ament 索引, 会报 "package 'my_robot' not found"。
# 装完之后自检:
#   xacro $(ros2 pkg prefix my_robot)/share/my_robot/urdf/my_robot.urdf.xacro > /tmp/my_robot.urdf
#   gz sdf -p /tmp/my_robot.urdf > /dev/null && echo "URDF OK"

ament_package()
