import math

from obstacle_avoidance.reactive import FORWARD, TURN, ReactiveController


def make_scan(fn, beams=360):
    inc = 2 * math.pi / beams
    amin = -math.pi
    return [fn(amin + i * inc) for i in range(beams)], amin, inc


def test_drives_forward_in_open_space():
    ctrl = ReactiveController()
    ranges, amin, inc = make_scan(lambda a: float('inf'))
    v, w, _ = ctrl.compute(ranges, amin, inc, 6.0)
    assert ctrl.state == FORWARD
    assert v > 0.0
    assert abs(w) < 1e-6


def test_turns_towards_open_side():
    ctrl = ReactiveController()
    # wall in front, obstacle on the right, open on the left
    def fn(a):
        if abs(a) < math.radians(25):
            return 0.4
        if -math.radians(80) <= a < 0:
            return 0.7
        return 5.0
    ranges, amin, inc = make_scan(fn)
    v, w, _ = ctrl.compute(ranges, amin, inc, 6.0)
    assert ctrl.state == TURN
    assert v == 0.0
    assert w > 0.0  # positive = counter-clockwise = left


def test_hysteresis_keeps_turning_until_clear():
    ctrl = ReactiveController(stop_distance=0.6, clear_distance=0.9)
    blocked, amin, inc = make_scan(lambda a: 0.5)
    ctrl.compute(blocked, amin, inc, 6.0)
    assert ctrl.state == TURN
    in_between, _, _ = make_scan(lambda a: 0.75)
    ctrl.compute(in_between, amin, inc, 6.0)
    assert ctrl.state == TURN
    clear, _, _ = make_scan(lambda a: 2.0)
    ctrl.compute(clear, amin, inc, 6.0)
    assert ctrl.state == FORWARD


def test_steers_away_from_closer_side():
    ctrl = ReactiveController()
    ranges, amin, inc = make_scan(lambda a: 1.0 if a > math.radians(30) else 4.0)
    _, w, _ = ctrl.compute(ranges, amin, inc, 6.0)
    assert w < 0.0  # obstacle on the left -> turn right
