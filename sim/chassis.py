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
