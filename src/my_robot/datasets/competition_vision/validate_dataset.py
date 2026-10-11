#!/usr/bin/env python3
"""Check the committed image and label pairs before training."""

from pathlib import Path

from PIL import Image


HERE = Path(__file__).resolve().parent


def check_task(task):
    total = 0
    for split in ('train', 'val'):
        images = sorted((HERE / task / 'images' / split).glob('*.png'))
        labels = HERE / task / 'labels' / split
        if len(images) != 3:
            raise ValueError(f'{task}/{split}: expected 3 images, got {len(images)}')
        for image in images:
            with Image.open(image) as opened:
                opened.verify()
            rows = (labels / f'{image.stem}.txt').read_text(encoding='utf-8').splitlines()
            expected = (4 if split == 'train' else 5) if task == 'person' else (1 if split == 'train' else 2)
            if len(rows) != expected:
                raise ValueError(f'{image}: expected {expected} boxes, got {len(rows)}')
            for row in rows:
                parts = row.split()
                if len(parts) != 5 or parts[0] != '0':
                    raise ValueError(f'{image}: invalid row {row}')
                x, y, width, height = map(float, parts[1:])
                if not (0 < width <= 1 and 0 < height <= 1 and
                        width / 2 <= x <= 1 - width / 2 and
                        height / 2 <= y <= 1 - height / 2):
                    raise ValueError(f'{image}: box outside image: {row}')
            total += len(rows)
    print(f'{task}: 6 images, {total} boxes')


if __name__ == '__main__':
    check_task('person')
    check_task('plate')
