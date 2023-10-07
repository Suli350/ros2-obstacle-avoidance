import math

import pytest

from obstacle_avoidance.world import World, point_segment_distance, ray_circle, ray_segment


def test_ray_hits_circle_front_surface():
    assert ray_circle(0, 0, 1, 0, 5, 0, 1) == pytest.approx(4.0)


def test_ray_misses_circle():
    assert ray_circle(0, 0, 1, 0, 5, 3, 1) is None
    assert ray_circle(0, 0, -1, 0, 5, 0, 1) is None  # behind


def test_ray_from_inside_circle():
    assert ray_circle(5, 0, 1, 0, 5, 0, 1) == pytest.approx(1.0)


def test_ray_segment():
    assert ray_segment(0, 0, 1, 0, 3, -1, 3, 1) == pytest.approx(3.0)
    assert ray_segment(0, 0, 1, 0, 3, 1, 3, 2) is None
    assert ray_segment(0, 0, 1, 0, 0, 1, 5, 1) is None  # parallel


def test_world_cast_uses_nearest_hit():
    world = World(10, 10, [(5, 5, 1)])
    assert world.cast(1, 5, 0.0, 20) == pytest.approx(3.0)       # circle
    assert world.cast(1, 5, math.pi, 20) == pytest.approx(1.0)   # left wall
    assert world.cast(1, 5, 0.0, 2.0) == pytest.approx(2.0)      # max range


def test_collision():
    world = World(10, 10, [(5, 5, 1)])
    assert world.collides(5, 6.1, 0.2)
    assert not world.collides(5, 6.5, 0.2)
    assert world.collides(0.1, 5, 0.2)


def test_point_segment_distance():
    assert point_segment_distance(0, 1, -1, 0, 1, 0) == pytest.approx(1.0)
    assert point_segment_distance(3, 0, -1, 0, 1, 0) == pytest.approx(2.0)


def test_from_flat_validates():
    with pytest.raises(ValueError):
        World.from_flat(10, 10, [1.0, 2.0])
