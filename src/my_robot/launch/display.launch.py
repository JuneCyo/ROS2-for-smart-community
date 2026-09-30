#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
只做 URDF/Xacro 建模校验: 不开 Gazebo, 直接用 RViz2 看模型
==========================================================
    ros2 launch my_robot display.launch.py                 # 带关节滑条(可手动转轮子)
    ros2 launch my_robot display.launch.py gui:=false      # 不带滑条, 关节保持 0 位
    ros2 launch my_robot display.launch.py enable_camera:=false

也可以先把模型导成 URDF 再检查:
    xacro $(ros2 pkg prefix my_robot)/share/my_robot/urdf/my_robot.urdf.xacro > /tmp/my_robot.urdf
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')

    declared = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否使用 joint_state_publisher_gui 关节滑条'),
        DeclareLaunchArgument('rviz', default_value='true', description='是否打开 RViz2'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
    ]

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

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
            parameters=[{'robot_description': robot_description}],
        ),
        # 有滑条: joint_state_publisher_gui; 没有滑条: 普通 joint_state_publisher(关节全 0)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(gui),
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(gui),
            output='screen',
        ),
       
    ])
