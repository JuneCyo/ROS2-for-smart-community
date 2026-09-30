#!/usr/bin/env python3
"""Map server + AMCL localization + Nav2 navigation for competition world."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_share = get_package_share_directory('my_robot')
    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    params = LaunchConfiguration('nav_params_file')
    map_file = LaunchConfiguration('map')
    sim_time = LaunchConfiguration('use_sim_time')
    lite_nav = LaunchConfiguration('lite_nav')
    common = {'params_file': params, 'use_sim_time': sim_time,
              'autostart': 'true', 'use_composition': 'False'}

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(
            robot_share, 'maps', 'competition_nav_map.yaml'),
            description='与当前 Gazebo world 对齐的地图 YAML'),
        DeclareLaunchArgument('nav_params_file', default_value=os.path.join(
            robot_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('lite_nav', default_value='false',
                              description='只运行自动路线需要的 Nav2 节点'),
        LogInfo(msg='[导航] 启动 map_server、AMCL、Nav2 各生命周期节点。'),
        LogInfo(msg='[导航] amcl_tf_bridge 持续发布 map→odom，避免停车后 TF 过期。'),
        LogInfo(msg=['[导航] 地图：', map_file]),
        LogInfo(msg='[导航] 单独启动定位导航时，定位稳定后运行：ros2 run my_robot navigate_traffic_lights.py --ros-args -p use_sim_time:=true'),
        LogInfo(msg='[导航] 轻量模式：planner_server + velocity_smoother；保留地图和 AMCL，关闭未使用的控制器/行为树。',
                condition=IfCondition(lite_nav)),
        Node(package='my_robot', executable='amcl_tf_bridge.py',
             name='amcl_tf_bridge', output='screen',
             parameters=[{'use_sim_time': sim_time}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch, 'localization_launch.py')),
            launch_arguments={**common, 'map': map_file}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch, 'navigation_launch.py')),
            launch_arguments=common.items(),
            condition=UnlessCondition(lite_nav)),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[params, {'use_sim_time': sim_time}],
             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
             condition=IfCondition(lite_nav)),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=[params, {'use_sim_time': sim_time}],
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')],
             condition=IfCondition(lite_nav)),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': sim_time, 'autostart': True,
                          'node_names': ['planner_server', 'velocity_smoother']}],
             condition=IfCondition(lite_nav)),
        Node(package='rviz2', executable='rviz2', name='nav_rviz', output='screen',
             arguments=['-d', os.path.join(robot_share, 'rviz', 'cartographer.rviz')],
             parameters=[{'use_sim_time': sim_time}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='my_robot', executable='map_obstacles_cloud.py',
             name='map_obstacles_cloud', output='screen',
             parameters=[{'use_sim_time': sim_time}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
