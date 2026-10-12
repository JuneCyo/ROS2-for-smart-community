"""Check board geometry and the rule for leaving a plate station."""

import math
import sys
import unittest
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
from types import SimpleNamespace


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))

from navigate_traffic_lights import (  # noqa: E402
    PERSON_ROUTE_INDICES, PERSON_STOPS, PLATE_ROUTE_INDICES, ROUTE,
    TrafficLightRoute, angle_diff,
)


def board_center(model_name):
    world = ET.parse(PACKAGE.parent / 'smart_community' / 'worlds'
                     / 'competition_world.sdf').getroot()
    include = next(item for item in world.findall('.//world/include')
                   if item.findtext('uri') == f'model://{model_name}')
    x, y, _, _, _, yaw = map(float, include.findtext('pose').split())
    model = ET.parse(PACKAGE.parent / 'smart_community' / 'models'
                     / model_name / 'model.sdf').getroot()
    dx, dy, *_ = map(float, model.findtext('.//visual/pose').split())
    return (x + math.cos(yaw) * dx - math.sin(yaw) * dy,
            y + math.sin(yaw) * dx + math.cos(yaw) * dy)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class FakePlateRoute:
    tick = TrafficLightRoute.tick

    def __init__(self, now_ns, votes):
        self.now_ns = now_ns
        self.index = PLATE_ROUTE_INDICES[0]
        self.plate_detected = True
        self.plate_detection_state = True
        self.plate_read_state = True
        self.plate_detection_pub = FakePublisher()
        self.plate_read_pub = FakePublisher()
        self.park_start_time = 0
        self.park_duration = 3.0
        self.plate_wait_log_interval = 10.0
        self.last_plate_wait_log_ns = 0
        self.plate_votes = Counter(votes)
        self.plate_cont = 3
        self.plate_facing_index = self.index
        self.path = [(0, 0)]
        self.saved = []
        self.stops = 0
        self.logs = []

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now_ns))

    def stop(self):
        self.stops += 1

    def get_logger(self):
        return SimpleNamespace(warn=self.logs.append, info=self.logs.append)

    def save_plate_result(self, *args):
        self.saved.append(args)
        return True


class RouteStopsTest(unittest.TestCase):
    def test_both_observations_are_between_the_real_boards(self):
        a_x, a_y = board_center('Person_A')
        b_x, b_y = board_center('Person_B')
        self.assertEqual(len(ROUTE), 16)
        self.assertEqual(PERSON_ROUTE_INDICES, (5, 6))
        self.assertLess(b_x, a_x)
        for stop in PERSON_STOPS:
            self.assertLess(b_x, stop[1])
            self.assertLess(stop[1], a_x)
            self.assertAlmostEqual(stop[2], a_y, delta=0.25)
            self.assertAlmostEqual(stop[2], b_y, delta=0.25)
        self.assertEqual(PERSON_STOPS[0][1:3], PERSON_STOPS[1][1:3])
        self.assertAlmostEqual(angle_diff(PERSON_STOPS[0][3], 0), 0)
        self.assertAlmostEqual(abs(angle_diff(PERSON_STOPS[1][3], 0)), math.pi)
        self.assertNotEqual(ROUTE[PLATE_ROUTE_INDICES[0] - 1][1:3],
                            ROUTE[PLATE_ROUTE_INDICES[0]][1:3])

    def test_plate_faces_are_forward_of_car_posters_and_clear_of_white_line(self):
        world = ET.parse(PACKAGE.parent / 'smart_community' / 'worlds'
                         / 'competition_world.sdf').getroot()
        for number in (1, 2, 3):
            with self.subTest(car=number):
                model_name = f'Car{number}'
                include = next(item for item in world.findall('.//world/include')
                               if item.findtext('uri') == f'model://{model_name}')
                car_y = float(include.findtext('pose').split()[1])
                model_dir = PACKAGE.parent / 'smart_community' / 'models' / model_name
                model = ET.parse(model_dir / 'model.sdf').getroot()
                face = model.find('.//visual[@name="license_plate"]')
                self.assertIsNotNone(face)
                face_y = float(face.findtext('pose').split()[1])
                width = float(face.findtext('geometry/box/size').split()[0])
                self.assertGreater(face_y, 0)
                self.assertGreaterEqual(width, 0.5)
                self.assertLess(car_y + face_y, -3.85)
                self.assertIsNone(model.find('.//collision[@name="license_plate"]'))
                self.assertTrue((model_dir / 'materials/textures/PlateFace.png').is_file())

    def test_unread_plate_does_not_advance_after_long_wait(self):
        route = FakePlateRoute(60_000_000_000, {})
        route.tick()
        self.assertEqual(route.index, PLATE_ROUTE_INDICES[0])
        self.assertEqual(route.saved, [])
        self.assertEqual(route.stops, 1)

    def test_two_matching_reads_save_before_advancing(self):
        route = FakePlateRoute(4_000_000_000, {'浙C08X8B': 2})
        route.tick()
        self.assertEqual(route.saved, [(1, '浙C08X8B', 'confirmed', 2)])
        self.assertEqual(route.index, PLATE_ROUTE_INDICES[0] + 1)
        self.assertFalse(route.plate_detected)
        self.assertEqual(route.plate_read_pub.messages, [False])

    def test_world_odom_alignment_continues_on_return_leg(self):
        route = SimpleNamespace(plate_stop_use_odom=True,
                                index=PLATE_ROUTE_INDICES[-1] + 1,
                                odom_pose=object(), pose=object())
        self.assertIs(TrafficLightRoute.route_pose(route), route.odom_pose)
        route.index = PERSON_ROUTE_INDICES[0]
        self.assertIs(TrafficLightRoute.route_pose(route), route.pose)

    def test_goal_alignment_survives_position_noise(self):
        route = SimpleNamespace(index=15, aligning_index=None)
        align = TrafficLightRoute.should_align
        self.assertTrue(align(route, 0.19))
        self.assertTrue(align(route, 0.23))
        self.assertFalse(align(route, 0.29))
        route.index = 14
        self.assertFalse(align(route, 0.23))


if __name__ == '__main__':
    unittest.main()
