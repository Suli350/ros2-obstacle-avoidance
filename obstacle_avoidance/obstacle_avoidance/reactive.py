"""Reactive obstacle avoidance logic, independent of ROS.

The scan is split into sectors (angles relative to the robot's heading):
    front        |a| <= front_half_angle
    front_left   front_half_angle < a <= side_angle
    front_right  -side_angle <= a < -front_half_angle

State machine:
    FORWARD : drive, steering gently away from the closer side
    TURN    : rotate in place towards the more open side until the front is clear
"""
import math

FORWARD, TURN = 'FORWARD', 'TURN'


def sector_min(ranges, angle_min, angle_increment, lo, hi, range_max):
    best = range_max
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_increment
        if lo <= a <= hi and math.isfinite(r) and r < best:
            best = r
    return best


class ReactiveController:

    def __init__(self, cruise_speed=0.5, turn_speed=1.2, stop_distance=0.6,
                 clear_distance=0.9, front_half_angle=math.radians(25),
                 side_angle=math.radians(80), steer_gain=1.0):
        self.cruise_speed = cruise_speed
        self.turn_speed = turn_speed
        self.stop_distance = stop_distance
        self.clear_distance = clear_distance
        self.front_half_angle = front_half_angle
        self.side_angle = side_angle
        self.steer_gain = steer_gain
        self.state = FORWARD
        self.turn_dir = 1.0

    def sectors(self, ranges, angle_min, angle_increment, range_max):
        fa, sa = self.front_half_angle, self.side_angle
        return {
            'front': sector_min(ranges, angle_min, angle_increment, -fa, fa, range_max),
            'left': sector_min(ranges, angle_min, angle_increment, fa, sa, range_max),
            'right': sector_min(ranges, angle_min, angle_increment, -sa, -fa, range_max),
        }

    def compute(self, ranges, angle_min, angle_increment, range_max):
        """Return (linear, angular, sectors)."""
        s = self.sectors(ranges, angle_min, angle_increment, range_max)

        if self.state == FORWARD and s['front'] < self.stop_distance:
            self.state = TURN
            # Commit to one direction so we do not dither left/right.
            self.turn_dir = 1.0 if s['left'] >= s['right'] else -1.0
        elif self.state == TURN and s['front'] > self.clear_distance:
            self.state = FORWARD

        if self.state == TURN:
            return 0.0, self.turn_dir * self.turn_speed, s

        # FORWARD: slow down as things get close, steer toward open space
        slow = min(1.0, (s['front'] - self.stop_distance) / max(1e-6, self.clear_distance))
        linear = self.cruise_speed * max(0.2, slow)
        balance = (1.0 / max(s['right'], 0.05)) - (1.0 / max(s['left'], 0.05))
        angular = max(-self.turn_speed, min(self.turn_speed, self.steer_gain * balance))
        return linear, angular, s
