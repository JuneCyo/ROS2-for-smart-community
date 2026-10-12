"""Regression checks using camera frames captured at the two board stops."""

import sys
import unittest
from pathlib import Path

import cv2
import torch
from ultralytics import YOLO

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))

from person_count import count_people  # noqa: E402


class PersonCountTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(2)
        cls.model = YOLO(str(PACKAGE / 'models' / 'person_yolov8n.pt'))

    def frame(self, name):
        image = cv2.imread(str(PACKAGE / 'tests' / 'fixtures' / name))
        self.assertIsNotNone(image)
        return image

    def test_board_stop_frames(self):
        self.assertEqual(count_people(self.model, self.frame('board_a_stop.png')), 4)
        self.assertEqual(count_people(self.model, self.frame('board_b_stop.png')), 5)

    def test_duplicate_box_at_board_b(self):
        image = self.frame('board_b_stop.png')
        height, width = image.shape[:2]
        matrix = cv2.getRotationMatrix2D((width / 2, height / 2), -2, 1.0)
        tilted = cv2.warpAffine(image, matrix, (width, height),
                                borderValue=(180, 180, 180))
        self.assertEqual(count_people(self.model, tilted), 5)


if __name__ == '__main__':
    unittest.main()
