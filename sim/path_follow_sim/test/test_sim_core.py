import math

from path_follow_sim import kinematics as kin
from path_follow_sim import scenarios
from path_follow_sim.metrics_node import summarize
from path_follow_sim.sdf_model import estimate, parse_sdf

import os
import pytest


def test_ackermann_circle_closes():
    wb, steer, v = 1.0, math.atan(0.25), 1.0
    body = kin.ackermann_to_body(v, steer, wb)
    assert body.w == pytest.approx(0.25)
    pose = kin.Pose2D()
    dt = 0.01
    for _ in range(int(2 * math.pi / body.w / dt)):
        pose = kin.integrate(pose, body, dt)
    assert math.hypot(pose.x, pose.y) < 1e-2


def test_four_ws_crab_and_counter_phase():
    crab = kin.four_ws_to_body(1.0, 0.2, 0.2, 0.5, 0.5)
    assert crab.w == pytest.approx(0.0)
    assert crab.vy == pytest.approx(math.tan(0.2))
    opp = kin.four_ws_to_body(1.0, 0.2, -0.2, 0.5, 0.5)
    assert opp.vy == pytest.approx(0.0)
    assert opp.w == pytest.approx(2 * math.tan(0.2))


def test_first_order_and_rate_limit():
    assert kin.first_order(0.0, 1.0, 0.1, 0.0) == 1.0
    assert 0.0 < kin.first_order(0.0, 1.0, 0.1, 0.5) < 1.0
    assert kin.rate_limit(0.0, 5.0, 1.0, 0.1) == pytest.approx(0.1)


@pytest.mark.parametrize('kind', sorted(scenarios.SCENARIOS))
def test_scenarios_are_dense_and_finite(kind):
    pts = scenarios.generate(kind, 0.05)
    assert len(pts) > 10
    gaps = [math.dist(a[:2], b[:2]) for a, b in zip(pts, pts[1:])]
    assert max(gaps) < 0.06
    assert all(math.isfinite(v) for p in pts for v in p)


def test_keep_heading_and_unknown():
    assert all(p[2] == 0.3 for p in scenarios.generate('lane_change', 0.1, 0.3))
    with pytest.raises(ValueError):
        scenarios.generate('nope')


def test_tracking_error_sign():
    path = scenarios.generate('line', 0.1, length=5.0)
    cte, he, s = scenarios.tracking_error(path, 2.0, 0.3, 0.1)
    assert cte == pytest.approx(0.3) and he == pytest.approx(0.1) and s == pytest.approx(2.0, abs=0.1)
    assert scenarios.tracking_error(path, 2.0, -0.3, 0.0)[0] == pytest.approx(-0.3)


def test_summarize_pass_and_fail():
    path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    limits = {'max_cross_track': 0.3, 'max_final_error': 0.1}
    good = [(0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.05, 0.0)]
    assert summarize(good, path, limits)['passed']
    bad = good[:1] + [(1.0, 0.5, 0.5, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0)]
    assert summarize(bad, path, limits)['failed_checks'] == ['max_cross_track', 'final_error']
    assert not summarize([], path, limits)['passed']


SDF = os.path.expanduser('~/.gazebo/models/cart_front_steer/model.sdf')


@pytest.mark.skipif(not os.path.exists(SDF), reason='gazebo model not installed')
def test_sdf_cart_front_steer():
    e = estimate(parse_sdf(SDF))
    assert e['chassis'] == 'ackermann'
    assert e['wheelbase'] == pytest.approx(1.0)
    assert e['track'] == pytest.approx(0.596)
    assert e['wheel_radius'] == pytest.approx(0.15)
