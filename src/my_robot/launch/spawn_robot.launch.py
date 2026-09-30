#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
往"已经跑起来的" Gazebo 里生成 my_robot (不再启动 Gazebo)
=========================================================
前提: 当前 Gazebo 必须加载了 gazebo_ros 的 factory 插件
      (即启动 gzserver 时带了 -s libgazebo_ros_factory.so,
       或直接用 ros2 launch gazebo_ros gazebo.launch.py / my_robot gazebo.launch.py)

    ros2 launch my_robot spawn_robot.launch.py
    ros2 launch my_robot spawn_robot.launch.py x:=3.0 y:=-4.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')

    declared = [
        DeclareLaunchArgument('x', default_value='3.95'),
        DeclareLaunchArgument('y', default_value='-4.00'),
        DeclareLaunchArgument('z', default_value='0.06'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('entity', default_value='my_robot'),
        DeclareLaunchArgument('spawn_delay', default_value='2.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
    ]

    robot_description = ParameterValue(
        Command([
            'xacro ', os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro'),
            ' enable_lidar:=', LaunchConfiguration('enable_lidar'),
            ' enable_camera:=', LaunchConfiguration('enable_camera'),
        ]),
        value_type=str)

    return LaunchDescription(declared + [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        TimerAction(
            period=LaunchConfiguration('spawn_delay'),
            actions=[Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_my_robot',
                output='screen',
                arguments=['-topic', 'robot_description',
                           '-entity', LaunchConfiguration('entity'),
                           '-x', LaunchConfiguration('x'),
                           '-y', LaunchConfiguration('y'),
                           '-z', LaunchConfiguration('z'),
                           '-Y', LaunchConfiguration('yaw'),
                           '-timeout', '120.0'],
            )],
        ),
    ])
