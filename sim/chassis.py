"""Ideal planar chassis model. No ROS, network, or hardware commands."""
import math


class Chassis:
    def __init__(self):
        self.x = self.y = self.yaw = self.time = 0.0
        self.velocity = [0.0] * 3
        self.target = [0.0] * 3
        self.last_command = -math.inf

    def command(self, values):
        values = list(values)
        if len(values) != 3 or not all(math.isfinite(v) for v in values):
            raise ValueError('Expected three finite velocities: vx, vy, wz')
        # Conservative simulation limits, not manufacturer hardware limits.
        speed = math.hypot(*values[:2])
        scale = min(1.0, 0.2 / speed) if speed else 1.0
        self.target = [values[0] * scale, values[1] * scale,
                       max(-0.4, min(0.4, values[2]))]
        self.last_command = self.time

    def step(self, dt):
        if not math.isfinite(dt) or not 0 < dt <= 0.05:
            raise ValueError('dt must be in (0, 0.05] seconds')
        self.time += dt
        if self.time - self.last_command > 1.0:
            self.target = [0.0] * 3
        old = self.velocity[:]
        for i, (v, goal) in enumerate(zip(old, self.target)):
            # Decelerate to zero before reversing direction.
            reversing = v * goal < 0
            dest = 0.0 if reversing else goal
            rate = 0.6 if reversing or abs(dest) < abs(v) else 0.3
            self.velocity[i] = v + max(-rate * dt, min(rate * dt, dest - v))
        vx, vy, wz = [(a + b) / 2 for a, b in zip(old, self.velocity)]
        heading = self.yaw + wz * dt / 2
        self.x += (math.cos(heading) * vx - math.sin(heading) * vy) * dt
        self.y += (math.sin(heading) * vx + math.cos(heading) * vy) * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))


class Square:
    """Fixed-heading square using simulated position feedback, not wall time."""
    def __init__(self, chassis, side=0.2):
        if not math.isfinite(side) or side <= 0:
            raise ValueError('side must be positive')
        x, y = chassis.x, chassis.y
        self.points = [(x + side, y), (x + side, y + side), (x, y + side), (x, y)]
        self.index = 0

    def tick(self, chassis):
        if self.index == len(self.points):
            chassis.command([0, 0, 0])
            return True
        x, y = self.points[self.index]
        dx, dy = x - chassis.x, y - chassis.y
        distance = math.hypot(dx, dy)
        if distance < 0.002 and math.hypot(*chassis.velocity[:2]) < 0.005:
            self.index += 1
            chassis.command([0, 0, 0])
            return False
        speed = min(0.05, distance * 1.5)
        vx, vy = (speed * dx / distance, speed * dy / distance) if distance else (0, 0)
        c, s = math.cos(chassis.yaw), math.sin(chassis.yaw)
        chassis.command([c * vx + s * vy, -s * vx + c * vy, 0])
        return False


class PickRoute(Square):
    """Simulation only: forward, right, backward, left, left. Fixed heading."""
    def __init__(self, chassis, width, length):
        if not all(math.isfinite(v) and v > 0 for v in (width, length)):
            raise ValueError('width and length must be positive measured distances')
        c, s = math.cos(chassis.yaw), math.sin(chassis.yaw)
        # Diagram up = starting forward; diagram right = starting body -y.
        offsets = [(length, 0), (length, -width), (0, -width),
                   (0, -width / 2), (0, 0)]
        self.points = [(chassis.x + c*x - s*y, chassis.y + s*x + c*y)
                       for x, y in offsets]
        self.index = 0
        self.phase = 'moving'
        self.dwell_start = None

    def confirm_grasp(self):
        if self.phase != 'wait_grasp':
            return False
        self.phase = 'moving'
        return True

    def tick(self, chassis):
        if self.phase == 'wait_grasp':
            chassis.command([0, 0, 0])
            return False
        if self.phase == 'dwell':
            chassis.command([0, 0, 0])
            if math.hypot(*chassis.velocity[:2]) > 0.0001:
                self.dwell_start = None
            elif self.dwell_start is None:
                self.dwell_start = chassis.time
            elif chassis.time - self.dwell_start >= 15.0:
                self.phase = 'moving'
            return False
        previous = self.index
        done = super().tick(chassis)
        if self.index != previous:
            if self.index == 2:
                self.phase = 'wait_grasp'
            elif self.index == 4:
                self.phase = 'dwell'
        if done:
            self.phase = 'done'
        return done
