#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动竞赛场地、机器人状态发布器和 Gazebo 中的机器人。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Gazebo Classic 的 RTShaderLib 位于此目录。
GAZEBO_BUILTIN_RES = '/usr/share/gazebo-11'


def _share(pkg):
    """返回包的 share 目录；包不存在时返回 None。"""
    try:
        return get_package_share_directory(pkg)
    except Exception:
        return None


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    sc_share = _share('smart_community')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    sc_world = None
    if sc_share:
        candidate = os.path.join(sc_share, 'worlds', 'competition_world.sdf')
        if os.path.isfile(candidate):
            sc_world = candidate
    default_world = sc_world or os.path.join(gazebo_ros_share, 'worlds', 'empty.world')

    declared = [
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo world 文件'),
        # 默认位置在场地东南角的白线方框内。
        DeclareLaunchArgument('x', default_value='3.95',
                              description='机器人起始 x (白线方框中心)'),
        DeclareLaunchArgument('y', default_value='-4.00', description='机器人起始 y'),
        # 地面顶面 z=0.05；车体生成时留 1 cm 间隙。
        DeclareLaunchArgument('z', default_value='0.06', description='机器人起始 z'),
        DeclareLaunchArgument('yaw', default_value='1.5708', description='机器人起始偏航角(rad)'),
        DeclareLaunchArgument('gui', default_value='true', description='是否打开 Gazebo 客户端'),
        DeclareLaunchArgument('verbose', default_value='false', description='gzserver 详细输出'),
        DeclareLaunchArgument('rviz', default_value='false', description='是否同时打开 RViz2'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='使用仿真时间'),
        DeclareLaunchArgument('spawn_delay', default_value='8.0',
                              description='等 Gazebo 加载完再生成机器人(秒)'),
        DeclareLaunchArgument('enable_lidar', default_value='true', description='是否装激光雷达'),
        DeclareLaunchArgument('enable_camera', default_value='true', description='是否装相机'),
    ]

    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_delay = LaunchConfiguration('spawn_delay')

    # world 中的 model:// 引用需要这些模型目录。
    model_paths = []
    if sc_share:
        model_paths.append(os.path.join(sc_share, 'models'))
    model_paths.append(os.path.join(pkg_share, 'models'))
    model_paths.append(os.path.join(GAZEBO_BUILTIN_RES, 'models'))
    model_paths.append(os.path.expanduser('~/.gazebo/models'))
    if os.environ.get('GAZEBO_MODEL_PATH'):
        model_paths.append(os.environ['GAZEBO_MODEL_PATH'])
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', ':'.join([p for p in model_paths if p]))

    # ROS 插件可能覆盖 GAZEBO_RESOURCE_PATH，使 Gazebo 找不到 RTShaderLib。
    # 此时 /scan、/odom 正常，但相机不会发布图像；保留内置资源目录。
    res_paths = [GAZEBO_BUILTIN_RES]
    if os.environ.get('GAZEBO_RESOURCE_PATH'):
        res_paths.append(os.environ['GAZEBO_RESOURCE_PATH'])
    set_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH', ':'.join([p for p in res_paths if p]))

    plugin_paths = []
    if sc_share:
        plugin_paths.append(os.path.join(os.path.dirname(os.path.dirname(sc_share)), 'lib'))
    if os.environ.get('GAZEBO_PLUGIN_PATH'):
        plugin_paths.append(os.environ['GAZEBO_PLUGIN_PATH'])
    set_plugin_path = SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', ':'.join(plugin_paths))

    # 默认使用 vmwgfx；图形驱动异常时可设置 LIBGL_ALWAYS_SOFTWARE=1。
    set_software_gl = SetEnvironmentVariable(
        'LIBGL_ALWAYS_SOFTWARE', os.environ.get('LIBGL_ALWAYS_SOFTWARE', '0'))

    xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' enable_lidar:=', LaunchConfiguration('enable_lidar'),
            ' enable_camera:=', LaunchConfiguration('enable_camera'),
        ]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
    )

    start_gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world,
                          'verbose': verbose,
                          'init': 'true',      # 提供 /clock
                          'factory': 'true'}.items(),  # 提供 /spawn_entity
    )
    start_gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui),
    )

    spawn_robot = TimerAction(
        period=spawn_delay,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_my_robot',
                output='screen',
                arguments=['-topic', 'robot_description',
                           '-entity', 'my_robot',
                           '-x', x, '-y', y, '-z', z, '-Y', yaw,
                           '-timeout', '120.0'],
            ),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'my_robot.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(declared + [
        set_model_path,
        set_resource_path,
        set_plugin_path,
        set_software_gl,
        start_gzserver,
        start_gzclient,
        robot_state_publisher,
        spawn_robot,
        rviz_node,
    ])
