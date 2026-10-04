#!/usr/bin/env python3
"""One command: Gazebo robot, AMCL/Nav2 and the two-light route."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetLaunchConfiguration, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('my_robot'), 'launch')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    nav_rviz = LaunchConfiguration('nav_rviz_enabled')
    auto_route = LaunchConfiguration('auto_route')
    lite_nav = LaunchConfiguration('lite_nav')
    enable_camera = LaunchConfiguration('enable_camera')
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('auto_route', default_value='true'),
        DeclareLaunchArgument('lite_nav', default_value='true',
                              description='自动路线只启动 Nav2 规划器和速度平滑器'),
        DeclareLaunchArgument('enable_camera', default_value='true',
                              description='开启低分辨率相机用于红绿灯图像识别'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            get_package_share_directory('my_robot'), 'maps', 'competition_nav_map.yaml')),
        # Included launch files share configuration in Humble. Preserve the
        # requested RViz value before gazebo.launch.py sets its own to false.
        SetLaunchConfiguration('nav_rviz_enabled', rviz),
        LogInfo(msg='[1/3] Gazebo + robot_state_publisher + spawn_entity: 启动相机、激光和周期切换的两组红绿灯'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gazebo.launch.py')),
            launch_arguments={'gui': gui, 'rviz': 'false',
                              'enable_camera': enable_camera}.items()),
        TimerAction(period=10.0, actions=[
            LogInfo(msg='[2/3] map_server + AMCL + Nav2 + RViz: 启动定位和路径规划'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
                launch_arguments={'map': map_file, 'rviz': nav_rviz,
                                  'lite_nav': lite_nav}.items()),
            LogInfo(msg='[3/3] 自动路线 + 相机灯色识别 + 激光动态避障：绿灯通行，红黄灯停车',
                    condition=IfCondition(auto_route)),
            LogInfo(msg='[3/3] 自动路线已关闭；定位与 Nav2 已启动',
                    condition=UnlessCondition(auto_route)),
            Node(package='my_robot', executable='navigate_traffic_lights.py',
                 name='navigate_traffic_lights', output='screen',
                 parameters=[{'use_sim_time': True}], condition=IfCondition(auto_route)),
            Node(package='my_robot', executable='traffic_light_vision.py',
                 name='traffic_light_vision', output='screen',
                 parameters=[{'use_sim_time': True}], condition=IfCondition(auto_route)),
            # ========== 新增车牌识别节点 ==========
            Node(package='my_robot', executable='plate_detect_cn.py',
                 name='plate_detect_cn', output='screen',
                 parameters=[{'use_sim_time': True}]),
            Node(package='my_robot', executable='plate_recognize.py',
                 name='plate_recognizer', output='screen',
                 parameters=[{'use_sim_time': True}], condition=IfCondition(enable_camera)),
            # ======================================
        ]),
        LogInfo(msg='[操作] 手动路线：source install/local_setup.bash；ros2 run my_robot navigate_traffic_lights.py --ros-args -p use_sim_time:=true',
                condition=UnlessCondition(auto_route)),
        LogInfo(msg='[运行] 路线节点等待 /amcl_pose、/scan、Nav2 和红绿灯状态；障碍可绕则绕、空间不足就停车',
                condition=IfCondition(auto_route)),
        LogInfo(msg='[查看] ros2 topic echo /amcl_pose --once；ros2 action list；RViz Fixed Frame=map'),
    ])
