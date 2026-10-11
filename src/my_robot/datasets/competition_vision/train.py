#!/usr/bin/env python3
"""Fine-tune a detection weight without changing the runtime model files."""

import argparse
from pathlib import Path

import yaml
from ultralytics import YOLO


HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[3]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('task', choices=('person', 'plate'))
    parser.add_argument('--epochs', type=int)
    args = parser.parse_args()
    settings = yaml.safe_load((HERE / 'training.yaml').read_text(encoding='utf-8'))[args.task]
    model = (HERE / settings['model']).resolve()
    data = (HERE / settings['data']).resolve()
    if not model.is_file() or not data.is_file():
        raise FileNotFoundError(f'{model} or {data}')
    YOLO(str(model)).train(
        data=str(data), epochs=args.epochs or settings['epochs'],
        imgsz=settings['imgsz'], batch=settings['batch'], device=settings['device'],
        project=str(WORKSPACE / 'log/training'), name=args.task, exist_ok=True,
        workers=0,
    )


if __name__ == '__main__':
    main()
