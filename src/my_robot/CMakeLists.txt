cmake_minimum_required(VERSION 3.16)
project(my_robot)
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  launch
  rviz
  config
  maps
  tools
  doc
  models
 scripts/ultralytics
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/survey_driver.py
  scripts/navigate_traffic_lights.py
  scripts/amcl_tf_bridge.py
  scripts/map_obstacles_cloud.py
  scripts/traffic_light_vision.py
  scripts/plate_recognize.py
  scripts/plate_detect_cn.py
  scripts/person_count.py
  DESTINATION lib/${PROJECT_NAME}
)


# 配置阶段还查不到本包，xacro 必须等安装后再运行。
# 安装后可检查:
#   xacro $(ros2 pkg prefix my_robot)/share/my_robot/urdf/my_robot.urdf.xacro > /tmp/my_robot.urdf
#   gz sdf -p /tmp/my_robot.urdf > /dev/null && echo "URDF OK"

ament_package()

