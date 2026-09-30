--[[
  my_robot 的 Cartographer 2D 建图配置
  ====================================
  * 只用 360° 激光雷达 + 轮式里程计(无 IMU)
  * TF 树: map -> odom -> base_footprint -> lidar_link
    其中 odom->base_footprint 由 gazebo_ros_diff_drive 提供(世界坐标),
    map->odom 由 Cartographer 自己算并发布
  用法见 launch/cartographer.launch.py
]]
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_footprint",   -- 机器人本体(没有 IMU, 用底盘)
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = false,          -- odom 由 gazebo 插件提供, 不再重复发布
  publish_frame_projected_to_2d = true,

  use_odometry = true,                 -- 用轮式里程计做初值
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2        -- 本机只有 2 核

TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 4.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 场地只有 10x10 m, 走慢一点, 把运动滤波收紧一些, 保证闭环质量
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.4
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.04
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.15)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60

POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options
