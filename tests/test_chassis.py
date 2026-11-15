import math
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'sim'))
from chassis import Chassis, Square, PickRoute


class ChassisTests(unittest.TestCase):
    def test_invalid_input_does_not_refresh_watchdog(self):
        c = Chassis()
        for bad in ([1, 2], [float('nan'), 0, 0], [0, float('inf'), 0]):
            with self.assertRaises(ValueError): c.command(bad)
        self.assertEqual(c.last_command, -math.inf)

    def test_timeout_stops_and_does_not_restart(self):
        c = Chassis(); c.command([0.05, 0, 0])
        for _ in range(300): c.step(0.01)
        self.assertEqual(c.velocity, [0, 0, 0])
        x = c.x
        c.step(0.01)
        self.assertEqual(x, c.x)

    def test_body_frame_and_acceleration(self):
        c = Chassis(); c.yaw = math.pi / 2; c.command([0.05, 0, 0]); c.step(0.01)
        self.assertAlmostEqual(c.velocity[0], 0.003)
        self.assertAlmostEqual(c.x, 0)
        self.assertGreater(c.y, 0)

    def test_reverse_brakes_before_accelerating(self):
        c = Chassis(); c.velocity = [0.01, 0, 0]; c.command([-0.05, 0, 0])
        c.step(0.05)
        self.assertEqual(c.velocity[0], 0)
        c.step(0.05)
        self.assertLess(c.velocity[0], 0)

    def test_square_returns_to_start_at_two_step_sizes(self):
        for dt in (0.002, 0.01):
            c = Chassis(); s = Square(c)
            for _ in range(round(60 / dt)):
                s.tick(c); c.step(dt)
            self.assertEqual(s.index, 4)
            self.assertLess(math.hypot(c.x, c.y), 0.003)
            self.assertEqual(c.velocity, [0, 0, 0])

    def test_pick_route_waits_for_confirmation_and_full_dwell(self):
        c = Chassis(); route = PickRoute(c, 0.4, 0.2)
        self.assertFalse(route.confirm_grasp())
        for _ in range(5000):
            route.tick(c); c.step(0.01)
        self.assertEqual(route.phase, 'wait_grasp')
        self.assertEqual(route.index, 2)
        self.assertEqual(c.velocity, [0, 0, 0])
        self.assertTrue(route.confirm_grasp())
        dwell_start = None
        dwell_end = None
        for _ in range(10000):
            before = route.phase
            route.tick(c); c.step(0.01)
            if route.dwell_start is not None:
                dwell_start = route.dwell_start
            if before == 'dwell' and route.phase == 'moving':
                dwell_end = c.time
            if route.phase == 'done': break
        self.assertEqual(route.phase, 'done')
        self.assertGreaterEqual(dwell_end - dwell_start, 15)
        self.assertAlmostEqual(c.yaw, 0)
        self.assertLess(math.hypot(c.x, c.y), 0.003)


if __name__ == '__main__': unittest.main()
