# 智慧社区复赛工程代码

压缩包根目录是 ROS 2 工作空间。`src/smart_community` 包含 Gazebo 场景、模型和红绿灯插件；`src/my_robot` 包含机器人模型、建图、导航、避障及视觉识别源码。当前导航地图在 `src/my_robot/maps/competition_nav_map.yaml`，Cartographer 建图结果在根目录 `maps/`。两套视觉检测权重已放在 `src/my_robot/models/`。

## 环境与编译

适用环境为 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11。先安装两个包 `package.xml` 声明的 ROS 依赖，包括 Cartographer、Nav2、gazebo_ros 和 cv_bridge。将压缩包解压到一个空目录，进入该目录后执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select smart_community my_robot --symlink-install
source install/local_setup.bash
python3 -m venv --system-site-packages .venv-plate
source .venv-plate/bin/activate
python3 -m pip install -r src/my_robot/requirements-plate.txt \
  --extra-index-url https://download.pytorch.org/whl/cpu
```

## 一键运行

每次打开新终端，进入解压后的工作空间根目录，再执行：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
source .venv-plate/bin/activate
export ROS_LOG_DIR="$PWD/log/ros"
mkdir -p "$ROS_LOG_DIR"
ros2 launch my_robot bringup_navigation.launch.py
```

默认打开 Gazebo 与 RViz，小车依次经过两个红绿灯，在 A、B 立牌之间识别人数，再读取三辆车的车牌并返回起点。三辆车的整车贴图与碰撞体已一起移近巡检车道，车牌仍在原车身贴图上；导航地图对应新车位。无图形界面的测试可在启动命令后加 `gui:=false rviz:=false plate_gui:=false`。人数和车牌结果分别写入 `log/ros/person_results.csv`、`log/ros/plate_results.csv`。车牌站读到两次一致的完整文字并保存后，小车才驶向下一站。

## 单独建图

在已加载工作空间环境的终端依次运行：

```bash
ros2 launch my_robot gazebo.launch.py gui:=false
ros2 launch my_robot cartographer.launch.py rviz:=true
ros2 run my_robot survey_driver.py
bash "$(ros2 pkg prefix my_robot)/share/my_robot/tools/save_cartographer_map.sh"
```

建图和 AMCL 自动导航分别启动。其他路线、模型、数据集与光照配置见 `src/my_robot/README.md`、`src/my_robot/datasets/competition_vision/README.md` 和 `src/my_robot/LIGHTING.md`。
