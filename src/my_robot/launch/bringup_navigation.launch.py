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
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('my_robot'), 'launch')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    nav_rviz = LaunchConfiguration('nav_rviz_enabled')
    auto_route = LaunchConfiguration('auto_route')
    lite_nav = LaunchConfiguration('lite_nav')
    enable_camera = LaunchConfiguration('enable_camera')
    plate_gui = LaunchConfiguration('plate_gui')
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('auto_route', default_value='true'),
        DeclareLaunchArgument('lite_nav', default_value='true',
                              description='自动路线只启动 Nav2 规划器和速度平滑器'),
        DeclareLaunchArgument('enable_camera', default_value='true',
                              description='开启相机用于红绿灯、立牌人数与车牌识别'),
        DeclareLaunchArgument('plate_gui', default_value='true',
                              description='显示车牌相机窗口；无图形界面测试时设为 false'),
        DeclareLaunchArgument('lighting', default_value='normal',
                              description='Gazebo 场景光照：normal、dim 或 bright'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            get_package_share_directory('my_robot'), 'maps', 'competition_nav_map.yaml')),
        # Humble 共享子启动文件的配置；先保存 RViz 参数，避免被 Gazebo 覆盖。
        SetLaunchConfiguration('nav_rviz_enabled', rviz),
        LogInfo(msg='[1/3] Gazebo + robot_state_publisher + spawn_entity: 启动相机、激光和周期切换的两组红绿灯'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gazebo.launch.py')),
            launch_arguments={'gui': gui, 'rviz': 'false',
                              'lighting': LaunchConfiguration('lighting'),
                              'enable_camera': enable_camera}.items()),
        Node(package='my_robot', executable='plate_detect_cn.py',
             name='plate_detect_cn', output='screen',
             parameters=[{'use_sim_time': True,
                          'use_gui': ParameterValue(plate_gui, value_type=bool),
                          'defer_detection': ParameterValue(auto_route, value_type=bool)}],
             condition=IfCondition(enable_camera)),
        Node(package='my_robot', executable='plate_recognize.py',
             name='plate_recognizer', output='screen',
             parameters=[{'use_sim_time': True,
                          'defer_reading': ParameterValue(auto_route, value_type=bool)}],
             condition=IfCondition(enable_camera)),
        Node(package='my_robot', executable='person_count.py',
             name='person_counter', output='screen',
             parameters=[{'use_sim_time': True}], condition=IfCondition(enable_camera)),
        TimerAction(period=10.0, actions=[
            LogInfo(msg='[2/3] map_server + AMCL + Nav2 + RViz: 启动定位和路径规划'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
                launch_arguments={'map': map_file, 'rviz': nav_rviz,
                                  'lite_nav': lite_nav}.items()),
            LogInfo(msg='[3/3] 自动路线 + 灯色、立牌人数与车牌识别 + 激光动态避障',
                    condition=IfCondition(auto_route)),
            LogInfo(msg='[3/3] 自动路线已关闭；定位与 Nav2 已启动',
                    condition=UnlessCondition(auto_route)),
            Node(package='my_robot', executable='navigate_traffic_lights.py',
                 name='navigate_traffic_lights', output='screen',
                 parameters=[{'use_sim_time': True}], condition=IfCondition(auto_route)),
            Node(package='my_robot', executable='traffic_light_vision.py',
                 name='traffic_light_vision', output='screen',
                 parameters=[{'use_sim_time': True}], condition=IfCondition(auto_route)),
        ]),
        LogInfo(msg='[操作] 手动路线：source install/local_setup.bash；ros2 run my_robot navigate_traffic_lights.py --ros-args -p use_sim_time:=true',
                condition=UnlessCondition(auto_route)),
        LogInfo(msg='[运行] 路线节点等待 /amcl_pose、/scan、Nav2 和红绿灯状态；障碍可绕则绕、空间不足就停车',
                condition=IfCondition(auto_route)),
        LogInfo(msg='[查看] ros2 topic echo /amcl_pose --once；ros2 action list；RViz Fixed Frame=map'),
    ])
