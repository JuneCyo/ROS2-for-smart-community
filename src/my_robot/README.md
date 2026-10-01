# my_robot —— 轮式机器人 URDF/Xacro 建模 + Gazebo 仿真

> ROS 2 Humble + Gazebo Classic 11 ｜ 车体 0.30 × 0.20 × 0.08 m ｜ 两轮差速 + 前后万向轮
> 传感器：**360° 激光雷达**（`/scan`）+ **单目相机**（`/camera/image_raw`）

---

## 0. 定位导航：依次通过两个红绿灯

当前 `competition_world.sdf` 中，红绿灯 1 的通道中心在 `(4.0, 0.0)`，通道沿南北方向；红绿灯 2 的通道中心在 `(-2.0, 0.25)`，通道沿东西方向。闭环路线从 `(3.95, -4.0)` 朝北直行穿过第一个灯，到北端左转向西，再左转向南，在 `(1.05, 0.10)` 右转向西穿过第二个灯；到西端左转向南，在车牌前横向白线北侧的 `(-4.0, -2.90)` 左转，沿横向通道到东端 `(3.95, -2.90)` 右转返回起点，不经过车牌前白线。导航逐段读取 AMCL 位姿并调用 Nav2 `ComputePathToPose` 检查通路，沿指定车道中心行驶，避免规划器抄近路压线。场地原贴图把起点方框和车道完全封闭，`competition_world.sdf` 在其余必经白线上设有窄黑色通道；车牌前横向白线保持原样。这些通道只有视觉效果，不改变激光碰撞地图。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select smart_community my_robot
source install/local_setup.bash
ros2 launch my_robot bringup_navigation.launch.py
```

一键启动默认同时打开 Gazebo 和 RViz，并自动运行路线。RViz 在 Gazebo 启动约 10 秒后由导航阶段打开。若当前终端的 `DISPLAY` 为空，在本机图形桌面使用 `DISPLAY=:0 ros2 launch my_robot bringup_navigation.launch.py gui:=true rviz:=true auto_route:=true`。

2 核 / 4 GB 的 VMware 虚拟机默认采用轻量配置：使用可用的 `vmwgfx` OpenGL 加速；相机为 320×240、8 Hz，专门识别灯色；只启动定位、Nav2 路径规划器和速度平滑器；Gazebo 物理更新目标 500 Hz，RViz 目标帧率设为 45 FPS 且默认关闭 TF 可视化。RViz 用 `/map_obstacles` 点云显示地图占用栅格，绕开当前 VMware 驱动下 RViz Map 插件的 GLSL 着色器错误；AMCL 和 Nav2 仍直接使用原始 `/map`。这些配置写在启动文件、场景和 RViz 配置中，重启虚拟机后继续生效。GNOME 桌面动画也已关闭并持久保存。需要完整 Nav2 `NavigateToPose` 服务时加 `lite_nav:=false`；若 VMware 图形驱动异常，可在命令前加 `LIBGL_ALWAYS_SOFTWARE=1` 回退。

这一条启动命令会在终端输出 `[1/3]`、`[2/3]`、`[3/3]` 提示：

| 阶段 | 启动的节点/功能 | 关键输出 |
|---|---|---|
| 1 | Gazebo、`traffic_signal_plugin`、`robot_state_publisher`、`spawn_entity` | `/clock`、`/scan`、`/camera/image_raw`、`/odom`、两个灯的 `/traffic_light/lightN/state` |
| 2 | `map_server`、`amcl`、`amcl_tf_bridge`、Nav2 `planner_server`、`velocity_smoother`、`map_obstacles_cloud`、RViz | `/map`、`/map_obstacles`、`/amcl_pose`、`map→odom`、`/compute_path_to_pose` |
| 3 | `traffic_light_vision.py`、`navigate_traffic_lights.py` | 图像识别发布 `/traffic_light/lightN/vision_state`；红黄灯停车、绿灯通行，激光发现障碍后在车道内绕行或停车；完成 11 个航点返回起点 |

虚拟机省资源时用 `gui:=false rviz:=false`。只启动定位导航、不自动跑路时用 `auto_route:=false`；确认 `/amcl_pose` 后，在新终端运行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
ros2 run my_robot navigate_traffic_lights.py --ros-args -p use_sim_time:=true
```

也可分开启动，以便检查每一层：

```bash
# 终端 1：场地、机器人和传感器
ros2 launch my_robot gazebo.launch.py
# 终端 2：静态地图、AMCL、Nav2、RViz（不要同时启动 Cartographer）
ros2 launch my_robot navigation.launch.py
# 终端 3：定位可用后，启动双红绿灯路线
ros2 run my_robot navigate_traffic_lights.py --ros-args -p use_sim_time:=true
```

`amcl_tf_bridge` 先用配置中的初始位姿和 `/odom` 发布 `map→odom`，收到 `/amcl_pose` 后改用 AMCL 定位位姿持续刷新；这样启动时即使 AMCL 首次发布较早，也不会卡在等待 TF。当前 Humble 仿真中 AMCL 自带 TF 广播会在行驶中停止刷新，导致 Nav2 报 `Transform data too old`。配置中已关闭 AMCL 自带广播，避免同一 TF 出现两个发布者。

路线脚本的 `start_at` 参数可从指定航点恢复，取值 `0..10`，默认 `0`；仅在车已位于对应路段时使用。`speed` 默认 `0.18` m/s。每段有 Nav2 通路检查和车道偏离检查；激光在前方约 1.25 m 发现地图中没有的障碍时，先检查两侧是否有足够空间，再在路线中心左右最多约 0.38 m 内绕行；两侧不安全或距离小于 0.32 m 时停车，障碍移走后继续。绕行候选轨迹还会查询地面贴图的白线位置和场景里的黑色通行缺口，要求车体外缘与白线至少留约 0.03 m；无法满足就停车，因此不会为躲障碍去压车牌前白线。红绿灯插件在 Gazebo Classic 中按红 5 秒、黄 2 秒、绿 5 秒切换外观和状态话题；相机节点按灯架几何位置裁剪图像识别灯色。路线在灯前同时参考图像与仿真状态，冲突时停车，图像暂时看不见灯时使用状态话题。2026-10-01 双窗口配置测试完成 11/11 个航点并返回起点，两个红灯前均出现停车等待日志；在北向直线路段临时放入半径 0.07 m 的障碍后，记录到向左绕行并最终完成 11/11 航点。障碍突然出现在车前约 0.4 m 时，小车停车，移走障碍后继续。2026-09-30 无相机轻量配置的 `/clock` 实时率约 0.97、RViz 状态栏约 44 FPS；启用相机和灯色识别后负载会提高，实际数值随宿主机变化。

定位与导航自检：

```bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 action list | grep compute_path_to_pose
```

导航默认加载 `my_robot/maps/competition_nav_map.yaml`。仓库原有 `maps/smart_community_map.yaml` 是此前 SLAM 生成的地图，其障碍物位置与**当前** `competition_world.sdf` 不一致；新地图由 `tools/generate_navigation_map.py` 按当前 SDF 的激光高度碰撞轮廓生成，坐标与 Gazebo 一致。更改场景后可重新生成：

```bash
python3 src/my_robot/tools/generate_navigation_map.py \
  src/smart_community/worlds/competition_world.sdf \
  src/smart_community/models src/my_robot/maps
colcon build --packages-select my_robot
source install/setup.bash
```

旧的 `smart_community/scripts/traffic_timer.py` 使用 Gazebo Sim 接口，不参与当前 Gazebo Classic 启动；实际切灯由 `traffic_signal_plugin` 完成。图像识别依赖当前场地中的两个已知灯架位置，换场地时需更新 `traffic_light_vision.py` 的 `LIGHTS` 坐标和相机几何参数。

---

## 0.1 两条基础命令

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot
source install/setup.bash

# ① 一键仿真：Gazebo 竞赛场地 + 机器人（生成在白线方框内）+ 可选 RViz
ros2 launch my_robot gazebo.launch.py

# ② 只看模型（不开 Gazebo，纯 URDF 检查，带关节滑条）
ros2 launch my_robot display.launch.py
```

Gazebo 里机器人应该出现在**地面白色方框内**：

```
白线方框范围:  x ∈ [3.26, 4.73] ,  y ∈ [-4.75, -3.33]      (方框中心 ≈ (4.0, -4.0))
默认生成位姿:  x = 3.95 , y = -4.00 , z = 0.06 , yaw = 1.5708(朝北, 面向场地里面)
```

> 注意：**x=3.0 其实在方框左边线上（方框左边界 x=3.26）**，车体（0.30 m 长）
> 若放在 x=3.0 会压在/越过白线。默认取 `x=3.95`（方框中心）保证整台车都在白线内；
> 若确实要放在 x=3.0：`ros2 launch my_robot gazebo.launch.py x:=3.0 y:=-4.0`
> 起始位姿全部是启动参数，`world/x/y/z/yaw` 都可以在命令行覆盖。

---

## 0.2 全部启动命令（速查）

```bash
cd ~/ros2_ws
colcon build                     # 或 colcon build --packages-select my_robot smart_community
source install/setup.bash

# ① 仿真：场地 + 机器人（白线方框里，朝北）
ros2 launch my_robot gazebo.launch.py
# ①' 想同时看 RViz（红色激光 + 相机图像 + 机器人模型）
ros2 launch my_robot gazebo.launch.py rviz:=true
# ①'' 无界面（省 CPU，建图/跑脚本时用）
ros2 launch my_robot gazebo.launch.py gui:=false

# ② 键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# ③ 只看机器人模型（不开 Gazebo）
ros2 launch my_robot display.launch.py

# ④ Cartographer 全场景建图（终端 2）
ros2 launch my_robot cartographer.launch.py rviz:=true
# ④' 自动跑遍全场（终端 3），也可以换成上面②的键盘遥控
ros2 run my_robot survey_driver.py
# ④'' 存图（终端 4）→ ~/ros2_ws/maps/smart_community_map.{pbstream,pgm,yaml,png}
bash $(ros2 pkg prefix my_robot)/share/my_robot/tools/save_cartographer_map.sh

# ⑤ 事后在 RViz 里看已经建好的地图
ros2 launch my_robot view_map.launch.py
#    (默认读 ~/ros2_ws/maps/smart_community_map.pbstream；
#     也可 ros2 launch my_robot view_map.launch.py map_yaml:=$HOME/ros2_ws/maps/smart_community_map.yaml)
```

---

## 1. 目录结构

```
my_robot/
├── package.xml / CMakeLists.txt
├── urdf/
│   ├── my_robot.urdf.xacro     # 总装：底盘 + 驱动轮 + 万向轮 + 传感器 + Gazebo 插件
│   └── sensors.xacro           # 传感器宏：my_lidar_360 / my_camera
├── launch/
│   ├── gazebo.launch.py        # ★ Gazebo(竞赛场地) + 机器人 + 可选 RViz
│   ├── display.launch.py       # 仅 RViz2（URDF 建模校验）
│   └── spawn_robot.launch.py   # 只往已运行的 Gazebo 里生成机器人
├── rviz/my_robot.rviz          # RViz 配置（RobotModel / TF / LaserScan / Image）
├── tools/                      # 自检小工具
│   ├── overhead_cam.sdf        # 临时俯视相机（拍一张“机器人是否在白线方框内”）
│   └── grab_image.py           # 从任意图像话题抓一帧存 png
└── doc/                        # 现场实拍（俯拍方框 / 相机画面 / Gazebo 界面）
```

---

## 2. 模型结构（URDF 树）

| Link | 几何 | 说明 |
|---|---|---|
| `base_footprint` | 无 | 车体在地面的投影，**也是 Gazebo 模型原点**（生成时的 x,y,z 就是它） |
| `base_link` | box 0.30×0.20×0.08 | 底盘车身，离地 0.055 m |
| `left_wheel_link` / `right_wheel_link` | cylinder r=0.035 w=0.020 | 驱动轮，轮距 0.22 m，`continuous` 关节 |
| `caster_front_link` / `caster_rear_link` | sphere r=0.012 | 万向轮（固定关节 + 极低摩擦） |
| `lidar_link` | cylinder r=0.035 h=0.040 | 车顶激光雷达，扫描平面离地 ≈ 0.12 m |
| `camera_link` | box 0.028×0.060×0.028 | 车头相机，沿 **+x** 方向上仰约 23° |

坐标系 / TF 树：

```
odom ──(diff_drive 插件)──> base_footprint ──(固定关节)──> base_link
                                                            ├── left_wheel_link   (continuous)
                                                            ├── right_wheel_link  (continuous)
                                                            ├── caster_front_link / caster_rear_link (fixed)
                                                            ├── lidar_link        (fixed)
                                                            └── camera_link       (fixed)
```

**几何上唯一的硬约束**：轮心离地高度必须等于轮半径（万向轮同理），
即 `wheel_z = -(body_z - wheel_r)`、`caster_z = -(body_z - caster_r)`。
写错会让车体压在轮子上，表现为「`/cmd_vel` 有指令、车轮也在转，但车几乎不动」。

---

## 3. 话题 / 插件一览

| 方向 | 话题 | 类型 | 提供者 |
|---|---|---|---|
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` | `libgazebo_ros_diff_drive.so` |
| 发布 | `/odom` | `nav_msgs/Odometry` | 同上（含 `odom→base_footprint` TF） |
| 发布 | `/joint_states` | `sensor_msgs/JointState` | `libgazebo_ros_joint_state_publisher.so` |
| 发布 | `/scan` | `sensor_msgs/LaserScan` | `libgazebo_ros_ray_sensor.so`（360 线，10 Hz，0.05–12 m） |
| 发布 | `/camera/image_raw`, `/camera/camera_info` | `sensor_msgs/Image`, `CameraInfo` | `libgazebo_ros_camera.so`（320×240，8 Hz） |
| 发布 | `/robot_description`, TF | `std_msgs/String`, `tf2_msgs/TFMessage` | `robot_state_publisher` |

自检命令：

```bash
ros2 topic list | grep -E "scan|camera|odom|joint_states"
ros2 topic echo /scan --once | head -20
ros2 topic hz /scan
ros2 run tf2_tools view_frames.py          # 生成 frames.pdf 看 TF 树
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 键盘遥控（i/j/k/l）
```

---

## 4. 参数（全部可用 `ros2 launch ... 名:=值` 覆盖）

| 参数 | 默认 | 说明 |
|---|---|---|
| `world` | `smart_community/share/smart_community/worlds/competition_world.sdf` | 场地文件（取不到包时回退到 `empty.world`） |
| `x` / `y` | `3.95` / `-4.00` | 起始位置（白线方框中心） |
| `z` | `0.06` | 起始高度（地面 box 顶面 z=0.05，留 1 cm 余量轻轻落地） |
| `yaw` | `0.0` | 起始朝向（rad） |
| `gui` | `true` | 是否开 Gazebo 客户端；`gui:=false` 为无头模式 |
| `rviz` | `false` | 是否同时开 RViz2 |
| `spawn_delay` | `8.0` | 等 Gazebo 加载完再生成实体（机器慢就调大） |
| `enable_lidar` / `enable_camera` | `true` / `true` | 关掉可提升实时率 |

`urdf/my_robot.urdf.xacro` 里可改车体尺寸、轮径、轮距、雷达线数、相机分辨率与视场角等。

---

## 5. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `Package 'my_robot' not found` | 忘了 `colcon build --packages-select my_robot && source install/setup.bash` |
| Gazebo 里场地全灰/找不到 `model://xxx` | `GAZEBO_MODEL_PATH` 没有包含 `smart_community/models`（launch 已自动注入；手工跑 `gzserver` 时需自己 export） |
| 机器人掉到地面以下 | `spawn_delay` 太小、场地还没加载完 → 加大；或 `z` 给大一点 |
| 自动导航有 `/scan`、`/odom` 但没有图像话题 | 自动导航默认 `enable_camera:=true`；检查 Gazebo 图形日志、相机插件和 `ros2 topic hz /camera/image_raw` |
| 有指令但车不动 / 原地打滑 | 检查轮心离地是否等于轮半径；`/odom` 是否变化；轮子 `mu1/mu2` 是否够大 |
| 反复调试后话题有多个发布者、行为异常 | DDS 残留：`rm -f /dev/shm/*fastrtps* ; ros2 daemon stop && ros2 daemon start` |

---

## 6. 自检 / 取证（怎么证明"车确实停在白线方框里"）

`tools/` 下两个小工具，用**俯视相机拍一张照片**来核对（不依赖 Gazebo 界面截图）：

```bash
source ~/ros2_ws/install/setup.bash
P=$(ros2 pkg prefix my_robot)/share/my_robot

# ① 在方框正上方 1.6 m 生成一个临时俯视相机（往正下方看）
ros2 run gazebo_ros spawn_entity.py -file $P/tools/overhead_cam.sdf \
    -entity overhead_check -x 3.95 -y -4.0
# ② 抓一帧存图
python3 $P/tools/grab_image.py /overhead_check/image_raw /tmp/overhead.png
# ③ 用完删掉
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: overhead_check}"
```

`/tmp/overhead.png` 里应当看到：白色方框的轮廓，以及方框中央白色的机器人车体。
顺手也可以抓机器人自己的相机：

```bash
python3 $P/tools/grab_image.py /camera/image_raw /tmp/robot_cam.png
```

本包 `doc/` 里就是按上面步骤现场拍的（本机真实渲染，非示意图）：

| 文件 | 内容 |
|---|---|
| `doc/01_robot_inside_white_square.png` | 方框正上方俯拍：白色方框 + 框内的机器人（**"起始位置在方框内"的直接证据**） |
| `doc/02_robot_camera_view.png` | 机器人自带相机 `/camera/image_raw` 的画面（车前方可见方框的白色边线） |
| `doc/03_gazebo_gui.png` | Gazebo 主界面（场地贴图/白线正常渲染） |
| `doc/04_white_square_marker.png` | 上一个会话里用户手动插入的 unit_sphere(3.97,-4.01) 正落在方框中心，用作定位参考 |

### 实测记录（Ubuntu 22.04 + ROS 2 Humble + Gazebo 11.10.2，2 核虚拟机）

| 项目 | 实测结果 |
|---|---|
| 生成位姿 | `/odom` 报 `x=3.950, y=-4.000, z=0.050`（`gz model -m my_robot -p` 一致） |
| 落地位姿 | `base_footprint` 稳定在 z=0.04997 ≈ 地面 box 顶面 0.05，四个着地点等高、车体不拖地 |
| 静止漂移 | 生成后 90 s 位姿完全不变（接触参数修好之前会以 ~0.7 mm/s 自己往前溜，见下） |
| 前进 | `/cmd_vel.linear.x=0.1` → 4 s 位移 0.47 m，`/odom` 同步 |
| 转向 | `/cmd_vel.angular.z=0.5` → 3 s 后 yaw +2.41 rad（逆时针，方向正确） |
| 激光 | `/scan` 约 9.9 Hz，360 线，`angle_min=-π`、`angle_max=+π`、`range_max=12` |
| 相机 | 自动导航为 `/camera/image_raw` 320×240 rgb8、8 Hz，`/camera/camera_info` 正常；相机向上倾约 23° |
| TF | `odom → base_footprint → {base_link, left/right_wheel_link, lidar_link, camera_link}` |

两个踩过的坑（代码里都有注释）：

1. **`GAZEBO_RESOURCE_PATH` 被 ROS 的 gazebo 插件包污染** → Gazebo 丢弃内置资源路径
   `/usr/share/gazebo-11`，找不到 RTShaderLib，渲染初始化失败，于是 `/scan`、`/odom` 都正常
   但**没有任何图像话题**。`gazebo.launch.py` 里已把内置路径显式补回去。
2. **接触参数会让车自己慢慢溜**：轮子 `minDepth` 给 0.5 mm 时，车重只让轮子压入约 0.1 mm，
   这些接触被判为无效，整车实际由两个 `mu=0.02` 的万向轮支撑 → 约 0.7 mm/s 匀速前溜。
   修法：轮子 `minDepth=0`，万向轮摩擦从 0.02 提到 0.4（仍远小于驱动轮的 1.0，不影响转弯）。

---

## 7. 其它场地

```bash
# 空场地（只有地面）
ros2 launch my_robot gazebo.launch.py \
  world:=/opt/ros/humble/share/gazebo_ros/worlds/empty.world z:=0.02

# 智慧社区沙盘（另一个工作空间的场景）
ros2 launch my_robot gazebo.launch.py \
  world:=$HOME/smart_community_ws/src/smart_community_sim/worlds/smart_community.world
```

---

## 8. 按赛题平面图新建的模型：楼宇A/B/C 与 四分类垃圾桶

模型放在 **smart_community** 包里（和地面/围墙同属一个场景包），**没有改动**已有的
地面、围墙、车（车牌）、人偶、红绿灯，只是在 `worlds/competition_world.sdf` 末尾
**追加**了 4 个 `<include>`（有 `自动生成` 注释标记，可反复重新生成）。

```
smart_community/
├── tools/gen_buildings_bins.py      # ★ 生成器（贴图+SDF+world include 全自动）
├── models/building_a|b|c/           # 楼宇：1.30 x 0.85 x 1.10 m，静态
│   └── model.sdf / model.config / materials/{facade_*.jpg, sign_*.png, building.material}
└── models/trash_bins/               # 四分类垃圾桶（1 个模型 4 个桶）
    └── model.sdf / model.config / materials/{label_*.png, trash_bins.material}
```

### 8.1 平面图 → 场地坐标的换算

平面图(以及地面贴图 map1.jpg)的外墙框：**列 7..625、行 12..629 == 场地 x,y ∈ [-5,+5]**，
并且**图片上方 = y 负方向(南)**，所以

```
x = -5 + (col - 7) * 10/618
y = -5 + (row - 12) * 10/617
```

用两个斑马线 + 起点区白框交叉校对过（误差 < 0.3 m）。据此量出的位置：

| 对象 | 平面图位置 | 世界坐标 | 尺寸(m) |
|---|---|---|---|
| 楼宇A | 楼名标签中心 | (1.18, **-1.44**) | 1.30 × 0.85 × 1.10 |
| 楼宇B | 楼名标签中心 | (1.18, **-0.28**) | 1.30 × 0.85 × 1.10 |
| 楼宇C | 楼名标签中心 | (1.18, **0.95**) | 1.30 × 0.85 × 1.10 |
| 垃圾桶(4 个) | 楼宇C 北侧一排 | x = **1.07**, y = **1.75 / 2.23 / 2.71 / 3.19** | 每个 0.62 × 0.46 × 0.92 |

### 8.1.1 实际摆放位置（按用户要求改过两版，以现在为准）

用户最终要求：**「从西往东依次坐落，放在东西方向那两条长白线之间」**，标签/招牌**朝北**。
这两条长白线是 **y = -3.31**（x -3.37..1.88）和 **y = -1.89**（x -3.37..0.46）之间的走廊
（高 1.42 m，西端是竖白线 x≈-3.34，东端到停车列白线 x=3.28）——
正好在小车起点 (3.95, -4, 朝北) 的**左前方**、红绿灯西边、停车位对面。

| 对象 | 世界坐标（当前） | 尺寸(m) |
|---|---|---|
| 楼宇A | **(-2.70, -2.60)** | 1.22 × 0.85 × 1.10 |
| 楼宇B | **(-1.40, -2.60)** | 1.22 × 0.85 × 1.10 |
| 楼宇C | **(-0.10, -2.60)** | 1.22 × 0.85 × 1.10 |
| 有害垃圾(粉红) | **(0.90, -2.60)** | 0.62 × 0.46 × 0.92 |
| 可回收物(青蓝) | **(1.60, -2.60)** | 同上 |
| 厨余垃圾(绿) | **(2.30, -2.60)** | 同上 |
| 其他垃圾(黄) | **(3.00, -2.60)** | 同上 |

也就是由西往东一条线：**楼宇A → 楼宇B → 楼宇C → 有害 → 可回收 → 厨余 → 其他**，
整排中心 y = -2.60，正好落在两条长白线正中（南北各留 ~0.28 m），
x 从 -3.31 一直排到 +3.31（西端贴着竖白线，东端到停车列白线）。
招牌/标签**朝北(+y)**，同时在**朝南(-y)** 面也各有一块（东西向一排时南北两面都露着，
小车从哪边过都能看到）。

垃圾桶颜色顺序与平面图一致：(有害)粉红 → (可回收)青蓝 → (厨余)绿 → (其他)黄。

### 8.2 重新生成 / 改尺寸 / 改位置

```bash
cd ~/ros2_ws/src/smart_community
python3 tools/gen_buildings_bins.py --apply-world   # 重新生成模型并把 include 写进 world
cd ~/ros2_ws && colcon build --packages-select smart_community && source install/setup.bash
```

改位置就编辑脚本顶部的 `BUILDINGS` / `BIN_X` / `BIN_Y` / `BIN_SIZE` 再跑一次即可（幂等）。

### 8.3 三个踩过的坑

1. **贴图文件名必须全局唯一**：Ogre 是按"文件名"在所有已注册资源目录里找贴图的，
   三栋楼都叫 `sign.png` 时会被解析成同一个文件（实测三栋楼全显示"楼宇A"）。
   生成器现在用 `sign_building_a.png` / `facade_building_a.jpg`。
2. **`<pose>` 最后一位是 yaw，不是"朝向编号"**：生成器早期把"招牌朝向=1"当 yaw 写进
   include，结果三栋楼各转了 1 rad，俯视图上呈现"阶梯状"。
3. **贴图朝向**：一开始以为盒子 +y 面要用左右镜像贴图，用侧视相机从南/北两面各拍一张才确认——
   盒子各个面的 UV 都是"从外面看"的，**+x / +y(北) / -y(南) 用同一张正常贴图文字都是正的**，
   给哪个面换镜像版哪个面就反。生成器现在只用一张正常贴图，不再生成镜像版。

---

## 9. Cartographer 全场景建图（SLAM）

配置：`config/cartographer_2d.lua`（纯激光 + 轮式里程计，无 IMU；`map → odom → base_footprint`）。

### 9.1 四条命令（4 个终端）

```bash
# 终端1：仿真（建图时建议关掉 GUI 省 CPU）
cd ~/ros2_ws && source install/setup.bash
ros2 launch my_robot gazebo.launch.py gui:=false

# 终端2：Cartographer + RViz —— RViz 里能实时看到地图一点点长出来
ros2 launch my_robot cartographer.launch.py rviz:=true

# 终端3：自动跑遍全场（不嫌累也可以换成键盘遥控）
ros2 run my_robot survey_driver.py
#        ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 终端4：跑完后存图 → ~/ros2_ws/maps/smart_community_map.{pbstream,pgm,yaml,png}
bash $(ros2 pkg prefix my_robot)/share/my_robot/tools/save_cartographer_map.sh
```

### 9.2 事后只看地图（不建图）

```bash
ros2 launch my_robot view_map.launch.py                      # 读 .pbstream
ros2 launch my_robot view_map.launch.py map_yaml:=$HOME/ros2_ws/maps/smart_community_map.yaml
# 只开 RViz（地图话题已经有人在发时）
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix my_robot)/share/my_robot/rviz/cartographer.rviz
```

RViz 配置 `rviz/cartographer.rviz`：Fixed Frame = `map`，显示 `Map`(/map，Transient Local)、
`LaserScan`(/scan)、`RobotModel`；TF 可视化默认关闭、需要时可手动打开，视角是 TopDownOrtho（俯视）。

### 9.3 survey_driver.py 的参数

```bash
ros2 run my_robot survey_driver.py --ros-args -p speed:=0.3 -p loops:=2 -p route:=clockwise
```
| 参数 | 默认 | 说明 |
|---|---|---|
| `speed` | 0.30 | 前进速度 m/s |
| `turn_speed` | 0.8 | 转向角速度 rad/s |
| `goal_tol` | 0.18 | 到点判定半径 m |
| `stop_dist` | 0.35 | 前方障碍急停距离 m |
| `loops` | 1 | 跑几圈（重复整条路线） |
| `route` | `default` | `default`(逆时针) / `clockwise` |

路线已按平面图避开：三辆车(y=-4)、人偶立牌(x≈2.5, y 1.2~2.8)、
**红绿灯龙门架立柱(x=3.27 / 4.73 与 -1.27 / -2.73)**、楼宇、垃圾桶；
东侧走 x=3.9（正好从红绿灯两根柱子中间穿过，横梁在 1.85 m 头顶）。
另外带反应式避障 + 20 s 卡死倒车脱困。

### 9.4 实测结果

| 项 | 结果 |
|---|---|
| Cartographer | 16 个子图(submap)，激光 9.85 Hz、里程计 50 Hz |
| 地图 | 206 × 206 @ 0.05 m ≈ 10.3 × 10.3 m，**覆盖 10×10 全场**（四面围墙 + 3 栋楼 + 4 个垃圾桶都在图上） |
| 产物 | `~/ros2_ws/maps/smart_community_map.pbstream / .pgm / .yaml / .png` |
| 回看 | `view_map.launch.py` 两种方式都实测能发布 /map |

现场实拍（本包 `doc/`）：

| 文件 | 内容 |
|---|---|
| `doc/06_field_with_buildings.png` | 全场俯拍 |
| `doc/07_buildings_bins_row.png` | **东西向那两条长白线之间的走廊俯拍**：由西往东 楼宇A/B/C + 4 个垃圾桶 |
| `doc/08_signs_north.png` | 侧视相机从北面拍：朝北招牌"楼宇C"等文字正常 |
| `doc/09_signs_south.png` | 侧视相机从南面拍：朝南招牌文字同样正常（未镜像） |
| `doc/10_cartographer_map.png` | Cartographer 建出来的地图 |

> ⚠️ 地图坐标系 `map` 是以**建图起始时的车身朝向为 x 轴**的。当前默认起始 `yaw=1.5708`（朝北），
> 所以地图相对世界坐标整体转了 90°（RViz 里看很正常，只是和世界坐标不对齐）。
> 想让地图和世界坐标一致：把起始 yaw 设成 0 再建图
> （`ros2 launch my_robot gazebo.launch.py yaw:=0`）。

