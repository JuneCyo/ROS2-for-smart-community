#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
在 RViz 里查看已经存好的地图(不重新建图)
=========================================
    ros2 launch my_robot view_map.launch.py
    ros2 launch my_robot view_map.launch.py map_pbstream:=$HOME/ros2_ws/maps/smart_community_map.pbstream

地图来源二选一:
  * map_pbstream 不为空 -> 用 cartographer_pbstream_map_publisher 直接读 .pbstream
  * map_yaml     不为空 -> 用 nav2_bringup 的 map_server 读 .yaml/.pgm
两者都发布 /map(latched), RViz 用 rviz/cartographer.rviz 显示。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

DEFAULT_DIR = os.path.expanduser('~/ros2_ws/maps')


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'cartographer.rviz')

    declared = [
        DeclareLaunchArgument('map_pbstream',
                              default_value=os.path.join(DEFAULT_DIR, 'smart_community_map.pbstream'),
                              description='Cartographer 的 pbstream 文件'),
        DeclareLaunchArgument('map_yaml', default_value='',
                              description='非空则改用 nav2 map_server 读 yaml/pgm'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    # map_yaml 非空 -> 走 nav2 map_server; 否则用 pbstream 发布
    has_yaml = PythonExpression(["'", LaunchConfiguration('map_yaml'), "' != ''"])
    use_pbstream = IfCondition(PythonExpression(["not (", has_yaml, ")"]))
    use_yaml = IfCondition(has_yaml)

    pbstream_pub = Node(
        package='cartographer_ros',
        executable='cartographer_pbstream_map_publisher',
        name='cartographer_pbstream_map_publisher',
        output='screen',
        condition=use_pbstream,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-pbstream_filename', LaunchConfiguration('map_pbstream')],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        condition=use_yaml,
        parameters=[{'yaml_filename': LaunchConfiguration('map_yaml'),
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        condition=use_yaml,
        parameters=[{'autostart': True,
                     'node_names': ['map_server'],
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription(declared + [pbstream_pub, map_server, lifecycle, rviz_node])
