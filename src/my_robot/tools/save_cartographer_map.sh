#!/usr/bin/env bash
# =============================================================================
# 保存 Cartographer 建图结果
#   bash save_cartographer_map.sh [输出目录] [地图名]
# 产物: <名>.pbstream (Cartographer 原始结果) + <名>.pgm/.yaml (Nav2 可直接用) + <名>.png
# =============================================================================
set -e
OUT_DIR="${1:-$HOME/ros2_ws/maps}"
NAME="${2:-smart_community_map}"
mkdir -p "$OUT_DIR"

source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash" || true

echo "== 1) 写 pbstream: $OUT_DIR/$NAME.pbstream =="
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$OUT_DIR/$NAME.pbstream', include_unfinished_submaps: true}"

echo "== 2) pbstream -> pgm + yaml =="
# 注意: 这个版本的 cartographer_pbstream_to_ros_map 只认 -pbstream_filename / -resolution,
# 输出固定是当前目录下的 map.pgm / map.yaml, 所以先 cd 过去再改名
cd "$OUT_DIR"
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename "$OUT_DIR/$NAME.pbstream" \
  -resolution 0.05
mv -f map.pgm "$NAME.pgm"
mv -f map.yaml "$NAME.yaml"
sed -i "s#image: map.pgm#image: $NAME.pgm#" "$NAME.yaml"

echo "== 3) 生成 png 预览 =="
python3 - "$OUT_DIR/$NAME.pgm" "$OUT_DIR/$NAME.png" <<'PY' || echo "(跳过 png: 缺 Pillow)"
import sys
from PIL import Image
im = Image.open(sys.argv[1])
im.save(sys.argv[2])
print('saved', sys.argv[2], im.size)
PY

echo "== 完成, 产物: =="
ls -la "$OUT_DIR"
echo
echo "RViz 里看这张地图(不建图时):"
echo "  ros2 run rviz2 rviz2 -d \$(ros2 pkg prefix my_robot)/share/my_robot/rviz/cartographer.rviz"
