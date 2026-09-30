#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
按赛题平面图生成「楼宇A/B/C」与「四分类垃圾桶」两组 Gazebo 模型, 并把 <include> 追加进 world
=========================================================================================
平面图 -> 场地坐标的换算(与地面贴图 map1.jpg 完全对齐):
    外墙框: 贴图/平面图 列 7..625, 行 12..629  ==  场地 x,y ∈ [-5,+5]
    注意: 贴图在 Gazebo 里的朝向是「图片上方 = y 负方向(南)」, 所以
        x = -5 + (col - 7) * 10 / 618
        y = -5 + (row - 12) * 10 / 617
    (用两条斑马线 + 起点方框交叉校对, 误差 < 0.3 m)

摆放位置(按用户要求): 东西向那两条长白线之间的走廊, 由西往东一字排开
    长白线        y = -3.31 (x -3.37..1.88) 与 y = -1.89 (x -3.37..0.46)
    走廊西端      竖白线 x ≈ -3.34  |  东端 停车列白线 x = 3.28
    => 整排中心 y = -2.60; 由西往东: 楼宇A(-2.70) 楼宇B(-1.40) 楼宇C(-0.10)
       有害(0.90) 可回收(1.60) 厨余(2.30) 其他(3.00)
       这条走廊就在小车起点(3.95,-4,朝北)的左前方, 红绿灯西边、停车位对面

生成结果:
    models/building_a|b|c/   楼宇(1.30 x 0.85 x 1.10 m, 静态, 带窗立面 + 楼名招牌)
    models/trash_bins/       4 个分类垃圾桶(有害/可回收/厨余/其他, 静态)

用法(在包目录或任意目录均可):
    python3 gen_buildings_bins.py                 # 只生成模型
    python3 gen_buildings_bins.py --apply-world   # 同时把 include 追加进 world
    python3 gen_buildings_bins.py --world /path/to/xxx.sdf
"""
import argparse
import os

from PIL import Image, ImageDraw, ImageFont

# ---------------------------------------------------------------- 位置
# 场地范围 x,y ∈ [-5,5]; 图片上方 = -y(南)
# 目标位置(按用户要求): 东西方向那两条长白线之间的走廊, 由西往东一字排开
#   长白线 y = -3.31 (x -3.37..1.88) 与 y = -1.89 (x -3.37..0.46) —— 走廊高 1.42 m
#   走廊西端是竖白线 x ≈ -3.34, 东端到停车列白线 x = 3.28
#   这条走廊就在小车起点(3.95,-4,朝北)的左前方, 红绿灯西边、停车位对面
ROW_Y = 2.60                     # 整排的中心 y(走廊正中)
# 由西往东: 楼宇A → 楼宇B → 楼宇C → 有害 → 可回收 → 厨余 → 其他
BUILDINGS = [
    ("building_a", "楼宇A", (-2.70, ROW_Y), (1.22, 0.85, 1.10)),
    ("building_b", "楼宇B", (-1.40, ROW_Y), (1.22, 0.85, 1.10)),
    ("building_c", "楼宇C", (-0.10, ROW_Y), (1.22, 0.85, 1.10)),
]

# 4 个垃圾桶也排在这一行里, 接在楼宇C 东边(模型坐标下的 x 偏移, 模型整体再放到 (0, ROW_Y))
BIN_X = [0.90, 1.60, 2.30, 3.00]
BIN_SIZE = (0.62, 0.46, 0.92)     # 长x, 宽y, 高z
# 顺序与平面图一致: 粉红(有害) 青蓝(可回收) 绿(厨余) 黄(其他)
BINS = [
    ("hazardous", "有害垃圾", (0.86, 0.28, 0.42)),
    ("recycle",   "可回收物", (0.10, 0.62, 0.70)),
    ("kitchen",   "厨余垃圾", (0.35, 0.68, 0.42)),
    ("other",     "其他垃圾", (0.93, 0.72, 0.22)),
]

# 贴图朝向的实测结论(用侧视相机分别从南/北两面拍证件照):
#   盒子各个面的 UV 都是"从外面看"的, 所以 +x / +y(北) / -y(南) 三个面
#   用同一张"正常"贴图, 文字都是正的; 一旦给某个面换成左右镜像版, 那个面就反了。
#   所以这里不需要镜像贴图(早期以为 +y 面要镜像, 实测是错的)。


MARK_BEGIN = "<!-- ==== 自动生成: 楼宇A/B/C 与 垃圾桶 (gen_buildings_bins.py) ==== -->"
MARK_END = "<!-- ==== 自动生成结束 ==== -->"

FONT_CANDIDATES = [
    "/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
    "C:/Windows/Fonts/msyh.ttc",
    "C:/Windows/Fonts/simhei.ttf",
]


def load_font(size):
    for path in FONT_CANDIDATES:
        if os.path.isfile(path):
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                continue
    raise RuntimeError("找不到中文字体, 请把字体路径加进 FONT_CANDIDATES")


# ---------------------------------------------------------------- 贴图生成
def make_facade(path):
    """楼宇立面: 米色墙面 + 4x3 窗户 + 底部大门"""
    W = H = 512
    im = Image.new("RGB", (W, H), (206, 199, 186))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, W, 26], fill=(150, 143, 130))        # 顶部女儿墙
    d.rectangle([0, H - 40, W, H], fill=(120, 114, 104))    # 底部基座
    for r in range(3):
        for c in range(4):
            x0 = 34 + c * 118
            y0 = 60 + r * 128
            d.rectangle([x0 - 6, y0 - 6, x0 + 82 + 6, y0 + 78 + 6], fill=(232, 229, 222))
            d.rectangle([x0, y0, x0 + 82, y0 + 78], fill=(58, 76, 94))
            d.line([x0 + 41, y0, x0 + 41, y0 + 78], fill=(150, 165, 180), width=3)
            d.line([x0, y0 + 39, x0 + 82, y0 + 39], fill=(150, 165, 180), width=3)
    d.rectangle([196, H - 118, 316, H - 40], fill=(70, 84, 96))       # 大门
    d.rectangle([196, H - 118, 316, H - 40], outline=(238, 236, 230), width=5)
    im.save(path, quality=92)


def make_sign(path, text):
    """楼名招牌: 白底 + 深蓝字 + 边框"""
    W, H = 512, 160
    im = Image.new("RGB", (W, H), (245, 246, 248))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, W - 1, H - 1], outline=(28, 60, 110), width=8)
    d.rectangle([10, 10, W - 11, H - 11], outline=(120, 150, 190), width=3)
    font = load_font(104)
    tb = d.textbbox((0, 0), text, font=font)
    d.text(((W - (tb[2] - tb[0])) / 2 - tb[0], (H - (tb[3] - tb[1])) / 2 - tb[1]),
           text, font=font, fill=(20, 45, 90))
    im.save(path)


def make_bin_label(path, text, rgb):
    """垃圾桶分类标签: 白底 + 彩色边框 + 中文分类名"""
    W, H = 384, 192
    im = Image.new("RGB", (W, H), (250, 250, 250))
    d = ImageDraw.Draw(im)
    col = tuple(int(255 * c) for c in rgb)
    d.rectangle([0, 0, W - 1, H - 1], outline=col, width=10)
    font = load_font(76)
    tb = d.textbbox((0, 0), text, font=font)
    d.text(((W - (tb[2] - tb[0])) / 2 - tb[0], (H - (tb[3] - tb[1])) / 2 - tb[1]),
           text, font=font, fill=(35, 35, 35))
    im.save(path)


def mirrored(src, dst):
    """存一份左右翻转的贴图(Gazebo 盒子 +y 面的 UV 与 +x 面相反)"""
    Image.open(src).transpose(Image.FLIP_LEFT_RIGHT).save(dst)


# ---------------------------------------------------------------- 模型文件
def write_building(root, name, cn, pose, size):
    mdir = os.path.join(root, name)
    matdir = os.path.join(mdir, "materials")
    os.makedirs(matdir, exist_ok=True)

    # 贴图文件名必须全局唯一: Ogre 是按"文件名"在所有已注册资源目录里找贴图的,
    # 三个楼都叫 sign.png 的话会被解析成同一张图(实测三栋楼全显示"楼宇A")。
    facade_name = "facade_%s.jpg" % name
    sign_name = "sign_%s.png" % name
    make_facade(os.path.join(matdir, facade_name))
    make_sign(os.path.join(matdir, sign_name), cn)

    mat_name = name.title().replace("_", "")
    with open(os.path.join(matdir, "building.material"), "w", encoding="utf-8") as f:
        for mat, tex in ((mat_name + "/Facade", facade_name),
                         (mat_name + "/Sign", sign_name),
                         (mat_name + "/SignSouth", sign_name)):
            f.write("material %s\n{\n  technique\n  {\n    pass\n    {\n"
                    "      texture_unit\n      {\n        texture %s\n      }\n"
                    "    }\n  }\n}\n\n" % (mat, tex))

    lx, ly, lz = size
    sy = ly / 2 + 0.012      # 朝北(+y)的招牌, 用户指定的正面朝向
    ny = -(ly / 2 + 0.012)   # 朝南(-y)的招牌: 这一排是东西向的, 南北两面都露在外面
    sgnz = lz / 2 - 0.30
    sdf = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="building">
      <collision name="collision">
        <geometry><box><size>{lx} {ly} {lz}</size></box></geometry>
      </collision>
      <visual name="body">
        <geometry><box><size>{lx} {ly} {lz}</size></box></geometry>
        <material>
          <script>
            <uri>model://{name}/materials</uri>
            <name>{Name}/Facade</name>
          </script>
        </material>
      </visual>
      <visual name="roof">
        <pose>0 0 {roofz} 0 0 0</pose>
        <geometry><box><size>{rx} {ry} 0.06</size></box></geometry>
        <material>
          <ambient>0.34 0.35 0.37 1</ambient>
          <diffuse>0.34 0.35 0.37 1</diffuse>
        </material>
      </visual>
      <visual name="sign_north">
        <pose>0 {sy} {sgnz} 0 0 0</pose>
        <geometry><box><size>{sgnw} 0.02 {sgnh}</size></box></geometry>
        <material>
          <script>
            <uri>model://{name}/materials</uri>
            <name>{Name}/Sign</name>
          </script>
        </material>
      </visual>
      <visual name="sign_south">
        <pose>0 {ny} {sgnz} 0 0 0</pose>
        <geometry><box><size>{sgnw} 0.02 {sgnh}</size></box></geometry>
        <material>
          <script>
            <uri>model://{name}/materials</uri>
            <name>{Name}/SignSouth</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""".format(name=name, Name=mat_name, lx=lx, ly=ly, lz=lz,
           roofz=lz / 2 + 0.03, rx=lx + 0.06, ry=ly + 0.06,
           sy=sy, ny=ny, sgnz=sgnz, sgnw=lx * 0.86, sgnh=0.26)
    with open(os.path.join(mdir, "model.sdf"), "w", encoding="utf-8") as f:
        f.write(sdf)

    cfg = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>junecyo</name></author>
  <description>{cn}(东西向一排, 招牌朝北 + 朝南)</description>
</model>
""".format(name=name, cn=cn)
    with open(os.path.join(mdir, "model.config"), "w", encoding="utf-8") as f:
        f.write(cfg)
    return pose


def write_trash_bins(root):
    name = "trash_bins"
    mdir = os.path.join(root, name)
    matdir = os.path.join(mdir, "materials")
    os.makedirs(matdir, exist_ok=True)

    lx, ly, lz = BIN_SIZE
    links = []
    for i, (key, cn, rgb) in enumerate(BINS):
        x = BIN_X[i]
        label = "label_%s.png" % key
        make_bin_label(os.path.join(matdir, label), cn, rgb)
        r, g, b = rgb
        links.append("""    <link name="bin_{key}">
      <pose>{x} 0 0 0 0 0</pose>
      <collision name="col">
        <geometry><box><size>{lx} {ly} {lz}</size></box></geometry>
      </collision>
      <visual name="body">
        <pose>0 0 {bz} 0 0 0</pose>
        <geometry><box><size>{bx} {by} {bz2}</size></box></geometry>
        <material>
          <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
          <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
        </material>
      </visual>
      <visual name="lid">
        <pose>0 0 {lidz} 0 0 0</pose>
        <geometry><box><size>{lidw} {lidd} 0.07</size></box></geometry>
        <material>
          <ambient>{lr:.2f} {lg:.2f} {lb:.2f} 1</ambient>
          <diffuse>{lr:.2f} {lg:.2f} {lb:.2f} 1</diffuse>
        </material>
      </visual>
      <visual name="label_north">
        <pose>0 {laby} {labz} 0 0 0</pose>
        <geometry><box><size>{labw} 0.012 {labh}</size></box></geometry>
        <material>
          <script>
            <uri>model://{name}/materials</uri>
            <name>TrashBins/{Key}</name>
          </script>
        </material>
      </visual>
      <visual name="label_south">
        <pose>0 {laby_neg} {labz} 0 0 0</pose>
        <geometry><box><size>{labw} 0.012 {labh}</size></box></geometry>
        <material>
          <script>
            <uri>model://{name}/materials</uri>
            <name>TrashBins/{Key}South</name>
          </script>
        </material>
      </visual>
    </link>
""".format(key=key, x=x, lx=lx, ly=ly, lz=lz,
           bz=lz * 0.5 - 0.035, bz2=lz - 0.07, bx=lx, by=ly,
           lidz=lz - 0.035, lidw=lx + 0.03, lidd=ly + 0.03,
           laby=ly / 2 + 0.008, laby_neg=-(ly / 2 + 0.008), labz=lz * 0.62,
           labw=lx * 0.9, labh=0.20,
           r=r, g=g, b=b,
           lr=min(1.0, r + 0.25), lg=min(1.0, g + 0.25), lb=min(1.0, b + 0.25),
           name=name, Key=key.title()))
    sdf = """<?xml version="1.0" ?>
<!-- 4 分类垃圾桶: 与楼宇同一排(由西往东: 楼宇A/B/C → 有害/可回收/厨余/其他), 标签朝北 + 朝南 -->
<sdf version="1.6">
  <model name="trash_bins">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
%s  </model>
</sdf>
""" % "".join(links)
    with open(os.path.join(mdir, "model.sdf"), "w", encoding="utf-8") as f:
        f.write(sdf)

    with open(os.path.join(matdir, "trash_bins.material"), "w", encoding="utf-8") as f:
        for key, cn, rgb in BINS:
            for suffix in ("", "South"):
                f.write("material TrashBins/%s%s\n{\n  technique\n  {\n    pass\n    {\n"
                        "      texture_unit\n      {\n        texture label_%s.png\n      }\n"
                        "    }\n  }\n}\n\n" % (key.title(), suffix, key))

    cfg = """<?xml version="1.0"?>
<model>
  <name>trash_bins</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>junecyo</name></author>
  <description>四分类垃圾桶(东西向一排, 标签朝北 + 朝南)</description>
</model>
"""
    with open(os.path.join(mdir, "model.config"), "w", encoding="utf-8") as f:
        f.write(cfg)


# ---------------------------------------------------------------- 写进 world
def world_block():
    out = [MARK_BEGIN]
    for name, cn, (x, y), size in BUILDINGS:
        out.append("""<!-- %s -->
<include>
  <uri>model://%s</uri>
  <name>%s</name>
  <pose>%.3f %.3f 0 0 0 0</pose>
</include>""" % (cn, name, name, x, y))
    out.append("""<!-- 垃圾桶(4 分类): 与楼宇同一排, x = %.2f ~ %.2f, y = %.2f -->
<include>
  <uri>model://trash_bins</uri>
  <name>trash_bins</name>
  <pose>0 %.2f 0 0 0 0</pose>
</include>""" % (BIN_X[0], BIN_X[-1], ROW_Y, ROW_Y))
    out.append(MARK_END)
    return "\n".join(out)


def apply_world(world_path):
    with open(world_path, "r", encoding="utf-8") as f:
        text = f.read()
    if MARK_BEGIN in text:                      # 幂等: 先删旧块
        s = text.index(MARK_BEGIN)
        e = text.index(MARK_END) + len(MARK_END)
        text = text[:s] + text[e:]
    idx = text.rindex("</world>")
    text = text[:idx] + "\n" + world_block() + "\n\n" + text[idx:]
    with open(world_path, "w", encoding="utf-8") as f:
        f.write(text)
    print("已写入 world: %s" % world_path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--models-dir",
                    default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models"))
    ap.add_argument("--world",
                    default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                         "worlds", "competition_world.sdf"))
    ap.add_argument("--apply-world", action="store_true")
    a = ap.parse_args()

    root = os.path.abspath(a.models_dir)
    print("模型输出目录: %s" % root)
    for name, cn, pose, size in BUILDINGS:
        write_building(root, name, cn, pose, size)
        print("  + %-12s %s  pose=(%.2f, %.2f)" % (name, cn, pose[0], pose[1]))
    write_trash_bins(root)
    print("  + %-12s 4 分类垃圾桶  x=%s, y=%.2f" % ("trash_bins", BIN_X, ROW_Y))

    if a.apply_world:
        apply_world(os.path.abspath(a.world))
    else:
        print("\nworld 里需要追加的内容(或用 --apply-world 自动写入):\n")
        print(world_block())


if __name__ == "__main__":
    main()
