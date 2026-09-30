#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
my_robot 一键仿真启动
=====================
  1. 启动 Gazebo (默认加载 smart_community 的竞赛场地 competition_world.sdf,
     地面就是那张画着白色车道线/白线方框的贴图)
  2. 用 xacro 生成机器人模型 -> robot_state_publisher 发布 /robot_description 与 TF
  3. spawn_entity 把机器人放进白线方框内 (默认 x=3.95, y=-4.00 = 方框中心)
  4. 可选: 同时打开 RViz2

用法:
    ros2 launch my_robot gazebo.launch.py
    ros2 launch my_robot gazebo.launch.py gui:=false                 # 无界面(无头)
    ros2 launch my_robot gazebo.launch.py x:=3.0 y:=-4.0             # 自定义起始位姿
    ros2 launch my_robot gazebo.launch.py rviz:=true                 # 顺便开 RViz
    ros2 launch my_robot gazebo.launch.py world:=/path/to/xxx.world  # 换场地

检查:
    ros2 topic list | grep -E "scan|camera|odom"
    ros2 run teleop_twist_keyboard teleop_twist_keyboard             # 键盘遥控
"""
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

# Gazebo Classic 11 的内置资源路径(着色器 RTShaderLib 在这里), 见下面的说明
GAZEBO_BUILTIN_RES = '/usr/share/gazebo-11'


def _share(pkg):
    """取包 share 目录, 取不到返回 None(方便在缺包时给出友好回退)"""
    try:
        return get_package_share_directory(pkg)
    except Exception:
        return None


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    sc_share = _share('smart_community')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # ---------- 默认世界: 竞赛场地(地面贴图上有白线方框) ----------
    sc_world = None
    if sc_share:
        candidate = os.path.join(sc_share, 'worlds', 'competition_world.sdf')
        if os.path.isfile(candidate):
            sc_world = candidate
    default_world = sc_world or os.path.join(gazebo_ros_share, 'worlds', 'empty.world')

    # ---------- 启动参数 ----------
    declared = [
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo world 文件'),
        # 白线方框: x∈[3.26,4.73], y∈[-4.75,-3.33], 方框中心 ≈ (4.0,-4.0)
        DeclareLaunchArgument('x', default_value='3.95',
                              description='机器人起始 x (白线方框中心)'),
        DeclareLaunchArgument('y', default_value='-4.00', description='机器人起始 y'),
        # 地面是 10x10x0.1 的 box(顶面 z=0.05), 留 1cm 余量让轮子轻轻落地
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

    # ---------- Gazebo 资源路径 ----------
    # ① 模型路径: world 里用 model://xxx 引用模型, 必须能搜到 smart_community/models
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

    # ② ★资源路径(坑): Gazebo Classic 只要发现 GAZEBO_RESOURCE_PATH 非空, 就**丢弃**
    #    自己的内置资源路径 /usr/share/gazebo-11。而本机 ROS 的若干 gazebo 插件包
    #    (gazebo_no_physics_plugin / gazebo_planar_move_plugin ...) 会通过 env hook
    #    把 GAZEBO_RESOURCE_PATH 设成它们的 share 目录 —— 于是 Gazebo 找不到
    #    RTShaderLib 着色器, 渲染初始化失败, 症状是:
    #      [Err] RTShaderSystem.cc Unable to find shader lib
    #      [Err] CameraSensor.cc Unable to create CameraSensor
    #    表现为 /scan、/odom 都正常, 但**没有任何图像话题**。
    #    这里把内置路径显式补回去, 相机才有图。
    res_paths = [GAZEBO_BUILTIN_RES]
    if os.environ.get('GAZEBO_RESOURCE_PATH'):
        res_paths.append(os.environ['GAZEBO_RESOURCE_PATH'])
    set_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH', ':'.join([p for p in res_paths if p]))

    # ③ 当前 VMware 虚拟机的 vmwgfx OpenGL 加速可用。默认使用它，
    #    只有图形驱动出问题时才显式设置 LIBGL_ALWAYS_SOFTWARE=1 回退。
    set_software_gl = SetEnvironmentVariable(
        'LIBGL_ALWAYS_SOFTWARE', os.environ.get('LIBGL_ALWAYS_SOFTWARE', '0'))

    # ---------- 机器人描述 ----------
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

    # ---------- Gazebo: gzserver(带 gazebo_ros 系统插件) + gzclient ----------
    start_gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world,
                          'verbose': verbose,
                          'init': 'true',      # libgazebo_ros_init.so   -> /clock 等
                          'factory': 'true'}.items(),  # libgazebo_ros_factory.so -> /spawn_entity
    )
    start_gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui),
    )

    # ---------- 生成机器人 ----------
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

    # ---------- 可选 RViz2 ----------
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
        set_software_gl,
        start_gzserver,
        start_gzclient,
        robot_state_publisher,
        spawn_robot,
        rviz_node,
    ])
