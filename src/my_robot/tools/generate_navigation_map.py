#!/usr/bin/env python3
"""Generate a world-aligned 2-D occupancy map from Gazebo Classic SDF collisions.

Only collisions intersecting the lidar plane are rasterized. This keeps the
traffic-light crossbars (1.7 m above the robot) clear while retaining poles.
"""
import math
import os
import sys
import xml.etree.ElementTree as ET


RESOLUTION = 0.05
ORIGIN = -5.2
SIZE = 208
LIDAR_Z = 0.12


def pose(element):
    values = [float(v) for v in (element.findtext('pose') or '').split()]
    return (values + [0.0] * 6)[:6]


def compose(parent, child):
    x, y, z, _, _, yaw = parent
    cx, cy, cz, _, _, cyaw = child
    return (x + math.cos(yaw) * cx - math.sin(yaw) * cy,
            y + math.sin(yaw) * cx + math.cos(yaw) * cy,
            z + cz, 0.0, 0.0, yaw + cyaw)


def write_map(world_path, models_dir, output_dir):
    world = ET.parse(world_path).getroot().find('world')
    occupied = bytearray([254] * (SIZE * SIZE))
    shapes = 0

    for include in world.findall('include'):
        uri = include.findtext('uri', '')
        if not uri.startswith('model://'):
            continue
        model_path = os.path.join(models_dir, uri[8:], 'model.sdf')
        model = ET.parse(model_path).getroot().find('model')
        model_pose = compose(pose(include), pose(model))
        for link in model.findall('link'):
            link_pose = compose(model_pose, pose(link))
            for collision in link.findall('collision'):
                box = collision.find('geometry/box/size')
                if box is None:
                    continue
                sx, sy, sz = (float(v) for v in box.text.split())
                x, y, z, _, _, yaw = compose(link_pose, pose(collision))
                if not z - sz / 2 <= LIDAR_Z <= z + sz / 2:
                    continue
                half_diagonal = math.hypot(sx, sy) / 2
                low_x = max(0, int((x - half_diagonal - ORIGIN) / RESOLUTION) - 1)
                high_x = min(SIZE, int((x + half_diagonal - ORIGIN) / RESOLUTION) + 2)
                low_y = max(0, int((y - half_diagonal - ORIGIN) / RESOLUTION) - 1)
                high_y = min(SIZE, int((y + half_diagonal - ORIGIN) / RESOLUTION) + 2)
                cosine, sine = math.cos(yaw), math.sin(yaw)
                for iy in range(low_y, high_y):
                    wy = ORIGIN + (iy + 0.5) * RESOLUTION
                    row = SIZE - 1 - iy
                    for ix in range(low_x, high_x):
                        wx = ORIGIN + (ix + 0.5) * RESOLUTION
                        dx, dy = wx - x, wy - y
                        lx = cosine * dx + sine * dy
                        ly = -sine * dx + cosine * dy
                        if abs(lx) <= sx / 2 and abs(ly) <= sy / 2:
                            occupied[row * SIZE + ix] = 0
                shapes += 1

    os.makedirs(output_dir, exist_ok=True)
    pgm = os.path.join(output_dir, 'competition_nav_map.pgm')
    yaml = os.path.join(output_dir, 'competition_nav_map.yaml')
    with open(pgm, 'wb') as stream:
        stream.write(f'P5\n{SIZE} {SIZE}\n255\n'.encode('ascii'))
        stream.write(occupied)
    with open(yaml, 'w', encoding='utf-8') as stream:
        stream.write('image: competition_nav_map.pgm\n')
        stream.write(f'resolution: {RESOLUTION}\norigin: [{ORIGIN}, {ORIGIN}, 0.0]\n')
        stream.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    print(f'Wrote {yaml} and {pgm} from {shapes} collision boxes')


if __name__ == '__main__':
    if len(sys.argv) != 4:
        sys.exit('usage: generate_navigation_map.py WORLD_SDF MODELS_DIR OUTPUT_DIR')
    write_map(*sys.argv[1:])
