#!/usr/bin/env python3
"""Run the two detection weights against the committed validation images."""

import hashlib
import json
from datetime import date
from pathlib import Path

import yaml
from ultralytics import YOLO


HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[3]
SOURCE_GROUP = {'person': 'board_b', 'plate': 'gazebo_view'}


def main():
    settings = yaml.safe_load((HERE / 'training.yaml').read_text(encoding='utf-8'))
    results = {'date': date.today().isoformat(), 'image_size': 640, 'tasks': {}}
    for task, config in settings.items():
        weight = (HERE / config['model']).resolve()
        data = (HERE / config['data']).resolve()
        metrics = YOLO(str(weight)).val(
            data=str(data), imgsz=640, batch=2, device='cpu', workers=0,
            project=str(WORKSPACE / 'log/dataset_check'), name=task,
            exist_ok=True, plots=False, verbose=False,
        )
        val_images = sorted((HERE / task / 'images/val').glob('*.png'))
        labels = HERE / task / 'labels/val'
        results['tasks'][task] = {
            'weights': weight.name,
            'weight_sha256': hashlib.sha256(weight.read_bytes()).hexdigest(),
            'data': data.name,
            'validation_source': SOURCE_GROUP[task],
            'validation_images': len(val_images),
            'validation_boxes': sum(len((labels / f'{p.stem}.txt').read_text().splitlines())
                                    for p in val_images),
            'box_precision': round(float(metrics.box.mp), 4),
            'box_recall': round(float(metrics.box.mr), 4),
            'box_map50': round(float(metrics.box.map50), 4),
            'box_map50_95': round(float(metrics.box.map), 4),
        }
    output = HERE / 'validation_results.json'
    output.write_text(json.dumps(results, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()
