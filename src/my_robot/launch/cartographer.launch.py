#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动 Cartographer 2D 建图；仿真和巡航节点分别启动。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    config_dir = os.path.join(pkg_share, 'config')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'cartographer.rviz')

    declared = [
        DeclareLaunchArgument('rviz', default_value='true',
                              description='是否同时打开 RViz2 看建图过程'),
        DeclareLaunchArgument('resolution', default_value='0.05',
                              description='占用栅格地图分辨率(m)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('configuration_basename', default_value='cartographer_2d.lua'),
    ]

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-configuration_directory', config_dir,
                   '-configuration_basename', LaunchConfiguration('configuration_basename')],
        remappings=[('scan', '/scan')],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-resolution', LaunchConfiguration('resolution'),
                   '-publish_period_sec', '1.0'],
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

    return LaunchDescription(declared + [cartographer_node, occupancy_grid_node, rviz_node])
