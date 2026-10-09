import importlib.util
import math
import os
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))

from traffic_light_vision import detect_lamp
from vision_lighting import enhance_lighting


class LightingTests(unittest.TestCase):
    def test_normal_frame_is_unchanged(self):
        values = np.linspace(30, 230, 256, dtype=np.uint8)
        image = np.repeat(values[None, :, None], 128, axis=0)
        image = np.repeat(image, 3, axis=2)
        self.assertIs(enhance_lighting(image), image)

    def test_dark_and_bright_frames_move_toward_midtones(self):
        dark = np.full((128, 128, 3), 25, dtype=np.uint8)
        bright = np.full((128, 128, 3), 225, dtype=np.uint8)
        self.assertGreater(np.median(enhance_lighting(dark)), 25)
        self.assertLess(np.median(enhance_lighting(bright)), 225)

    def test_dark_walls_do_not_change_normal_exposure(self):
        normal = np.full((720, 960, 3), 20, dtype=np.uint8)
        normal[:340] = 178
        normal[340:560, :385] = 80
        self.assertIs(enhance_lighting(normal), normal)
        dim = normal.copy()
        dim[340:] = 6
        dim[340:560, :385] = 24
        self.assertIsNot(enhance_lighting(dim), dim)

    def test_traffic_lamp_under_dim_and_bright_conditions(self):
        for background, lamp in ((20, (0, 0, 255)),
                                 (5, (0, 0, 65)),
                                 (210, (100, 100, 255))):
            with self.subTest(background=background):
                image = np.full((720, 960, 3), background, dtype=np.uint8)
                cv2.circle(image, (480, 118), 13, lamp, -1)
                self.assertEqual(detect_lamp(image, 4.0, -1.5, math.pi / 2, 1), 'red')

        near_lamp = np.full((720, 960, 3), 20, dtype=np.uint8)
        cv2.circle(near_lamp, (427, 103), 32, (0, 255, 0), -1)
        self.assertEqual(detect_lamp(near_lamp, 3.95, -1.5, math.pi / 2, 1), 'green')

    def test_lighting_world_changes_only_light_values(self):
        spec = importlib.util.spec_from_file_location('gazebo_launch',
                                                     PACKAGE / 'launch' / 'gazebo.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        source = PACKAGE.parent / 'smart_community' / 'worlds' / 'competition_world.sdf'
        original = source.read_bytes()
        self.assertEqual(module._lighting_world(str(source), 'normal'), str(source))
        for level, ambient, diffuse in (
            ('dim', '0.12 0.12 0.12 1', '0.25 0.25 0.25 1'),
            ('bright', '0.75 0.75 0.75 1', '1 1 1 1'),
        ):
            with self.subTest(level=level):
                path = module._lighting_world(str(source), level)
                self.addCleanup(os.unlink, path)
                world = ET.parse(path)
                self.assertEqual(world.findtext('.//world/scene/ambient'), ambient)
                self.assertEqual(world.findtext(".//world/light[@name='sun']/diffuse"), diffuse)
                self.assertEqual(len(world.findall('.//world/include')),
                                 len(ET.fromstring(original).findall('.//world/include')))
        self.assertEqual(source.read_bytes(), original)


if __name__ == '__main__':
    unittest.main()
