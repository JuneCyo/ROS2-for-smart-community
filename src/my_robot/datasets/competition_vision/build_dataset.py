#!/usr/bin/env python3
"""Build the small, source-grouped detection dataset from project assets."""

from pathlib import Path
from PIL import Image, ImageEnhance


HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[3]
MODELS = WORKSPACE / 'src/smart_community/models'

PERSON_A = MODELS / 'Person_A/materials/textures/PersonA.jpg'
PERSON_B = MODELS / 'Person_B/materials/textures/PersonB.jpg'
CAR = MODELS / 'Car1/materials/textures/Car1.jpg'
PLATE_SCENE = HERE / 'sources/plate_stop_camera.png'

# Pixel boxes were checked against the original images, before brightness variants.
PERSON_A_BOXES = [(89, 34, 354, 627), (368, 30, 666, 628),
                  (697, 39, 926, 628), (970, 35, 1170, 625)]
PERSON_B_BOXES = [(6, 12, 242, 630), (319, 34, 476, 623),
                  (543, 16, 734, 627), (752, 64, 962, 629),
                  (1044, 30, 1242, 628)]
CAR_BOXES = [(174, 426, 368, 492)]
PLATE_SCENE_BOXES = [(113, 530, 198, 559), (440, 529, 523, 559)]
VARIANTS = {'normal': 1.0, 'dim': 0.58, 'bright': 1.35}


def yolo_row(box, size):
    x1, y1, x2, y2 = box
    width, height = size
    if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
        raise ValueError(f'box {box} is outside {size}')
    return f'0 {(x1 + x2) / (2 * width):.6f} {(y1 + y2) / (2 * height):.6f} ' \
           f'{(x2 - x1) / width:.6f} {(y2 - y1) / height:.6f}\n'


def add_family(task, split, family, source, boxes):
    image = Image.open(source).convert('RGB')
    labels = ''.join(yolo_row(box, image.size) for box in boxes)
    images_dir = HERE / task / 'images' / split
    labels_dir = HERE / task / 'labels' / split
    images_dir.mkdir(parents=True, exist_ok=True)
    labels_dir.mkdir(parents=True, exist_ok=True)
    for variant, gain in VARIANTS.items():
        output = image if gain == 1.0 else ImageEnhance.Brightness(image).enhance(gain)
        name = f'{family}_{variant}'
        output.save(images_dir / f'{name}.png', optimize=True)
        (labels_dir / f'{name}.txt').write_text(labels, encoding='utf-8')


def main():
    add_family('person', 'train', 'board_a', PERSON_A, PERSON_A_BOXES)
    add_family('person', 'val', 'board_b', PERSON_B, PERSON_B_BOXES)
    add_family('plate', 'train', 'car_texture', CAR, CAR_BOXES)
    add_family('plate', 'val', 'gazebo_view', PLATE_SCENE, PLATE_SCENE_BOXES)
    print('built 3 train and 3 val images for each detection task')


if __name__ == '__main__':
    main()
