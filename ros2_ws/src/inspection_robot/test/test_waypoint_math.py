"""Unit tests for pure-math helpers in waypoint_follower.

These tests stub out every ROS2 / sensor import so they run with plain pytest
(no ROS2 installation required).
"""
import math
import sys
import os
import tempfile
from unittest.mock import MagicMock

# --- ROS2 stubs (must come before any inspection_robot import) ---------------
import types as _types

class _FakeNode:
    """Minimal stand-in so 'class Foo(Node)' compiles without ROS2 installed."""
    def __init__(self, *a, **kw):
        pass

_rclpy_node_mod = _types.ModuleType('rclpy.node')
_rclpy_node_mod.Node = _FakeNode

for _mod in (
    'rclpy', 'rclpy.qos',
    'geometry_msgs', 'geometry_msgs.msg',
    'nav_msgs', 'nav_msgs.msg',
    'sensor_msgs', 'sensor_msgs.msg',
    'std_msgs', 'std_msgs.msg',
):
    sys.modules.setdefault(_mod, MagicMock())

sys.modules['rclpy.node'] = _rclpy_node_mod
# -----------------------------------------------------------------------------

from inspection_robot.waypoint_follower import angle_diff, quat_to_yaw, WaypointFollower


class TestAngleDiff:
    def test_same_angle(self):
        assert abs(angle_diff(1.0, 1.0)) < 1e-9

    def test_small_positive(self):
        assert abs(angle_diff(0.5, 0.3) - 0.2) < 1e-9

    def test_small_negative(self):
        assert abs(angle_diff(0.3, 0.5) - (-0.2)) < 1e-9

    def test_wraps_positive(self):
        d = angle_diff(3.1, -3.1)
        assert abs(d) <= math.pi

    def test_wraps_negative(self):
        d = angle_diff(-3.1, 3.1)
        assert abs(d) <= math.pi

    def test_half_pi(self):
        assert abs(angle_diff(math.pi / 2, 0.0) - math.pi / 2) < 1e-9

    def test_result_in_range(self):
        import random
        rng = random.Random(42)
        for _ in range(200):
            a = rng.uniform(-10, 10)
            b = rng.uniform(-10, 10)
            d = angle_diff(a, b)
            assert -math.pi <= d <= math.pi, f"angle_diff({a}, {b}) = {d} out of range"


class TestQuatToYaw:
    def test_identity(self):
        assert abs(quat_to_yaw(0, 0, 0, 1)) < 1e-9

    def test_yaw_90(self):
        s = math.sin(math.pi / 4)
        c = math.cos(math.pi / 4)
        assert abs(quat_to_yaw(0, 0, s, c) - math.pi / 2) < 1e-6

    def test_yaw_180(self):
        assert abs(abs(quat_to_yaw(0, 0, 1, 0)) - math.pi) < 1e-6

    def test_yaw_minus_90(self):
        s = math.sin(-math.pi / 4)
        c = math.cos(-math.pi / 4)
        assert abs(quat_to_yaw(0, 0, s, c) - (-math.pi / 2)) < 1e-6

    def test_yaw_45(self):
        s = math.sin(math.pi / 8)
        c = math.cos(math.pi / 8)
        assert abs(quat_to_yaw(0, 0, s, c) - math.pi / 4) < 1e-6


class TestLoadWaypoints:
    def test_loads_full_entry(self, tmp_path):
        yaml_content = (
            "waypoints:\n"
            "  - { label: 'spot_a', x: 1.0, y: 2.0, yaw: 0.5 }\n"
            "  - { label: 'spot_b', x: -1.5, y: 0.0, yaw: 3.14 }\n"
        )
        p = tmp_path / "wps.yaml"
        p.write_text(yaml_content)
        wps = WaypointFollower._load_waypoints(str(p))
        assert len(wps) == 2
        assert wps[0] == (1.0, 2.0, 0.5, 'spot_a')
        assert abs(wps[1][0] - (-1.5)) < 1e-9

    def test_empty_path_returns_empty(self):
        assert WaypointFollower._load_waypoints('') == []

    def test_default_yaw_is_zero(self, tmp_path):
        yaml_content = "waypoints:\n  - { label: 'x', x: 0.0, y: 0.0 }\n"
        p = tmp_path / "wps.yaml"
        p.write_text(yaml_content)
        wps = WaypointFollower._load_waypoints(str(p))
        assert wps[0][2] == 0.0

    def test_default_label_is_empty_string(self, tmp_path):
        yaml_content = "waypoints:\n  - { x: 1.0, y: 1.0, yaw: 0.0 }\n"
        p = tmp_path / "wps.yaml"
        p.write_text(yaml_content)
        wps = WaypointFollower._load_waypoints(str(p))
        assert wps[0][3] == ''
