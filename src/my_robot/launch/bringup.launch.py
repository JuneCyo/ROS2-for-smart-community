#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bringup.launch.py 一键启动，无延时
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')

    # 启动参数
    declare_gui = DeclareLaunchArgument('gui', default_value='false')
    declare_auto_survey = DeclareLaunchArgument('auto_survey', default_value='true')

    # 1. Gazebo仿真：rviz直接硬编码true，启动即自动打开RViz
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui'), 'rviz': 'true'}.items()
    )

    # 2. display模块：关掉它自己的RViz，避免双开
    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'display.launch.py')),
        launch_arguments={'rviz': 'false'}.items()
    )

    # 3. Cartographer建图：不用传rviz参数（gazebo已经开了）
    carto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'cartographer.launch.py')),
        launch_arguments={'rviz': 'false'}.items()
    )

    # 4. 自动遍历驱动
    survey = Node(
        package='my_robot',
        executable='survey_driver.py',
        name='survey_driver',
        output='screen',
        condition=IfCondition(LaunchConfiguration('auto_survey'))
    )

    return LaunchDescription([
        declare_gui,
        declare_auto_survey,
        gazebo,
        display,
        carto,
        survey
    ])

