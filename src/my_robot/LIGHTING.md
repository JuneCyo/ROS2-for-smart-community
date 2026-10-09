# Gazebo 光照抗干扰验证

相机识别覆盖红绿灯、两块人偶立牌和三处车牌。正常光照仍使用原始图像；画面过暗、过亮或反差不足时，识别节点会校正亮度。车牌文字首次 OCR 未确认时，再尝试另一种亮度处理结果。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select smart_community my_robot
source install/local_setup.bash
source .venv-plate/bin/activate
export ROS_LOG_DIR="$PWD/log/ros"
mkdir -p "$ROS_LOG_DIR"
ros2 launch my_robot bringup_navigation.launch.py lighting:=dim
```

把 `dim` 换成 `bright` 可测试强光；`normal` 是默认值。启动参数也可用于单独的 `gazebo.launch.py`。光照模式会从当前场景生成临时 SDF，只更改 Gazebo 的环境光和太阳光，不更改地图、模型位置或碰撞体。每次切换模式需重新启动 Gazebo。

快速检查相机与识别话题：

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /traffic_light/light1/vision_state --once
ros2 topic echo /plate_detect --once
ros2 topic echo /person_count/final --once --qos-durability transient_local
```

如果只想观察画面，可在启动命令末尾加 `auto_route:=false rviz:=false`，在 Gazebo 中手动移动机器人或使用键盘遥控。极端曝光导致颜色或字符像素完全丢失时，软件校正无法还原，路线会继续按现有的红绿灯状态话题停车或通行，并将未确认的识别结果记录为未确认。
