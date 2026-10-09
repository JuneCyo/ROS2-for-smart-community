# 智慧社区巡检仿真工程

本工作区包含 `smart_community`（Gazebo 场景与红绿灯插件）和 `my_robot`（机器人模型、建图、导航及视觉识别）两个 ROS 2 包。最新的完整技术报告见 [PDF](reports/2026_smart_community/智慧社区复赛技术报告.pdf) 和 [可编辑文档](reports/2026_smart_community/智慧社区复赛技术报告.docx)；此前的简版技术方案见 [PDF](docs/智慧社区复赛技术方案.pdf)，可编辑源文件见同目录的 HTML。

## 环境与编译

项目按 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11 编写。除 ROS 包外，车牌和人偶识别依赖 Python 包。首次安装时在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select smart_community my_robot
source install/local_setup.bash
python3 -m venv --system-site-packages .venv-plate
source .venv-plate/bin/activate
python3 -m pip install -r src/my_robot/requirements-plate.txt \
  --extra-index-url https://download.pytorch.org/whl/cpu
```

当前代码还依赖 `cartographer_ros`、Nav2、`gazebo_ros`、`cv_bridge` 等 ROS 包；具体声明见两个包的 `package.xml`。安装过虚拟环境后不必重复执行创建和安装步骤。

## 一键运行

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
source .venv-plate/bin/activate
export ROS_LOG_DIR="$PWD/log/ros"
mkdir -p "$ROS_LOG_DIR"
ros2 launch my_robot bringup_navigation.launch.py
```

默认打开 Gazebo 和 RViz，自动执行双红绿灯、两块人偶立牌、三处车牌的巡检路线。关闭图形界面可加 `gui:=false rviz:=false`；只启动定位与导航可加 `auto_route:=false`。人数和车牌结果分别追加到 `log/ros/person_results.csv`、`log/ros/plate_results.csv`。

## 单独建图

在各自已加载工作区环境的终端中，依次运行：

```bash
ros2 launch my_robot gazebo.launch.py gui:=false
ros2 launch my_robot cartographer.launch.py rviz:=true
ros2 run my_robot survey_driver.py
bash "$(ros2 pkg prefix my_robot)/share/my_robot/tools/save_cartographer_map.sh"
```

建图和 AMCL 自动导航分别启动，不在同一个仿真会话中同时运行。当前自动导航使用 `src/my_robot/maps/competition_nav_map.yaml`；`maps/smart_community_map.*` 是先前 Cartographer 建图留下的产物。详细参数与自检命令见 [my_robot 说明](src/my_robot/README.md)。
