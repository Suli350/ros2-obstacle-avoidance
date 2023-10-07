"""Pure-python 2D world: walls (segments), round obstacles (circles), ray casting.

No ROS imports here, so everything can be unit tested with pytest.
"""
import math


class World:

    def __init__(self, width=10.0, height=10.0, circles=None):
        self.width = width
        self.height = height
        self.circles = list(circles or [])  # (cx, cy, r)
        self.segments = [
            ((0.0, 0.0), (width, 0.0)),
            ((width, 0.0), (width, height)),
            ((width, height), (0.0, height)),
            ((0.0, height), (0.0, 0.0)),
        ]

    def add_wall(self, x1, y1, x2, y2):
        self.segments.append(((x1, y1), (x2, y2)))

    @staticmethod
    def from_flat(width, height, flat_circles, flat_walls=()):
        if len(flat_circles) % 3:
            raise ValueError('obstacles must be a flat list of x, y, radius triples')
        if len(flat_walls) % 4:
            raise ValueError('walls must be a flat list of x1, y1, x2, y2 quadruples')
        circles = [tuple(flat_circles[i:i + 3]) for i in range(0, len(flat_circles), 3)]
        world = World(width, height, circles)
        for i in range(0, len(flat_walls), 4):
            world.add_wall(*flat_walls[i:i + 4])
        return world

    # ------------------------------------------------------------------
    def cast(self, x, y, angle, max_range):
        """Distance to the first hit along a ray, or max_range if nothing is hit."""
        dx, dy = math.cos(angle), math.sin(angle)
        best = max_range
        for cx, cy, r in self.circles:
            t = ray_circle(x, y, dx, dy, cx, cy, r)
            if t is not None and t < best:
                best = t
        for (x1, y1), (x2, y2) in self.segments:
            t = ray_segment(x, y, dx, dy, x1, y1, x2, y2)
            if t is not None and t < best:
                best = t
        return best

    def collides(self, x, y, radius):
        """True if a disc of `radius` at (x, y) overlaps any obstacle or wall."""
        for cx, cy, r in self.circles:
            if math.hypot(x - cx, y - cy) < r + radius:
                return True
        for (x1, y1), (x2, y2) in self.segments:
            if point_segment_distance(x, y, x1, y1, x2, y2) < radius:
                return True
        return False


def ray_circle(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    for t in (-b - sq, -b + sq):
        if t >= 0.0:
            return t
    return None


def ray_segment(ox, oy, dx, dy, x1, y1, x2, y2):
    ex, ey = x2 - x1, y2 - y1
    denom = dx * ey - dy * ex
    if abs(denom) < 1e-12:
        return None  # parallel
    wx, wy = x1 - ox, y1 - oy
    t = (wx * ey - wy * ex) / denom   # distance along ray
    u = (wx * dy - wy * dx) / denom   # position along segment
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


def point_segment_distance(px, py, x1, y1, x2, y2):
    ex, ey = x2 - x1, y2 - y1
    length_sq = ex * ex + ey * ey
    if length_sq == 0.0:
        return math.hypot(px - x1, py - y1)
    u = max(0.0, min(1.0, ((px - x1) * ex + (py - y1) * ey) / length_sq))
    return math.hypot(px - (x1 + u * ex), py - (y1 + u * ey))
