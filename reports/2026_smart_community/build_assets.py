from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / 'assets'
ASSETS.mkdir(exist_ok=True)
CHINESE_FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(CHINESE_FONT)
plt.rcParams['font.family'] = font_manager.FontProperties(fname=CHINESE_FONT).get_name()
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['savefig.dpi'] = 180

shutil.copy2(ROOT.parents[1] / 'src/my_robot/doc/06_field_with_buildings.png', ASSETS / 'field.png')

fig, ax = plt.subplots(figsize=(11, 5.3))
ax.set_xlim(0, 11)
ax.set_ylim(0, 5.3)
ax.axis('off')
items = [
    (0.3, 3.3, 2.1, 1.2, '仿真场景与机器人\n相机／激光／里程计／灯色'),
    (3.0, 3.4, 2.0, 1.0, '地图与状态估计\n占据栅格／粒子定位'),
    (5.8, 3.4, 2.0, 1.0, '规划与路线控制\n航点／停车／转头'),
    (8.7, 3.4, 2.0, 1.0, '底盘执行\n速度平滑／差速驱动'),
    (3.0, 0.8, 2.0, 1.5, '视觉识别\n灯色／人数\n车牌框／字符'),
    (6.0, 0.9, 2.0, 1.3, '任务结果\n两站人数汇总\n三站车牌记录'),
]
for x, y, w, h, label in items:
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.1',
                                linewidth=1.2, edgecolor='#285b80', facecolor='#e8f2f7'))
    ax.text(x+w/2, y+h/2, label, ha='center', va='center', fontsize=10)
arrows = [((2.5, 3.9), (2.9, 3.9)), ((5.1, 3.9), (5.7, 3.9)),
          ((7.9, 3.9), (8.6, 3.9)), ((1.7, 3.2), (3.7, 2.4)),
          ((5.1, 1.7), (6.0, 1.7)), ((5.1, 2.1), (6.4, 3.3)),
          ((7.0, 3.3), (7.0, 2.3))]
for a, b in arrows:
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle='-|>', mutation_scale=14,
                                  linewidth=1.3, color='#315d76'))
fig.tight_layout()
fig.savefig(ASSETS / 'architecture.png', bbox_inches='tight')
plt.close(fig)

fig, ax = plt.subplots(figsize=(8.4, 4.1))
labels = ['基线', '中间运行', '最终完整运行']
values = [278.6, 184.4, 170.9]
colors = ['#8daec1', '#8daec1', '#2e789b']
bars = ax.barh(labels, values, color=colors, height=0.55)
ax.invert_yaxis()
ax.axvline(180, linestyle='--', color='#ad4d43', linewidth=1.5, label='三分钟目标：180 秒')
for bar, value in zip(bars, values):
    ax.text(value + 3, bar.get_y() + bar.get_height()/2, f'{value:.1f} 秒', va='center', fontsize=10)
ax.set_xlim(0, 310)
ax.set_xlabel('仿真时间／秒')
ax.legend(loc='lower right', frameon=False)
ax.spines[['top', 'right']].set_visible(False)
fig.tight_layout()
fig.savefig(ASSETS / 'timing.png', bbox_inches='tight')
plt.close(fig)

points = [(3.95,-4.0),(3.95,-2.65),(3.95,-0.85),(3.95,0.85),
          (3.95,2.20),(3.95,3.8),(1.05,3.8),(1.05,2.20),
          (1.05,0.10),(-2.0,0.10),(-4.0,0.10),(-4.0,-2.50),
          (-4.0,-2.50),(-2.6,-2.50),(-1.2,-2.50),(3.95,-2.50),(3.95,-4.0)]
fig, ax = plt.subplots(figsize=(8.4, 7.0))
ax.plot([p[0] for p in points], [p[1] for p in points], '-o', lw=1.8,
        markersize=3, color='#2e789b')
marks = [((3.95,-4),'起点／终点',(0.1,-0.35)),
         ((3.95,2.2),'A 立牌',(0.1,0.0)), ((1.05,2.2),'B 立牌',(-1.35,0.0)),
         ((-4,-2.5),'车牌 1',(-0.4,-0.45)),((-2.6,-2.5),'车牌 2',(-0.55,0.2)),
         ((-1.2,-2.5),'车牌 3',(-0.45,-0.45)),
         ((3.95,-0.5),'灯 1',(0.1,0)),((-1.1,0.1),'灯 2',(-0.5,0.2))]
for (x,y),label,(dx,dy) in marks:
    ax.annotate(label,(x,y),xytext=(x+dx,y+dy),fontsize=9)
ax.set_xlim(-5.2,5.2)
ax.set_ylim(-4.8,4.5)
ax.set_aspect('equal')
ax.set_xlabel('世界坐标 x／米')
ax.set_ylabel('世界坐标 y／米')
ax.grid(alpha=0.25)
ax.set_title('按航点顺序绘制；示意线不代表实际轨迹')
fig.tight_layout()
fig.savefig(ASSETS / 'route.png', bbox_inches='tight')
plt.close(fig)
