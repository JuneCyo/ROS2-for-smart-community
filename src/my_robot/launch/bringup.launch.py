#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动仿真、RViz、Cartographer 和可选的巡航建图节点。"""
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

    declare_gui = DeclareLaunchArgument('gui', default_value='false')
    declare_auto_survey = DeclareLaunchArgument('auto_survey', default_value='true')

    # RViz 由 Gazebo 启动文件统一打开，避免重复实例。
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui'), 'rviz': 'true'}.items()
    )

    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'display.launch.py')),
        launch_arguments={'rviz': 'false'}.items()
    )

    carto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'cartographer.launch.py')),
        launch_arguments={'rviz': 'false'}.items()
    )

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

