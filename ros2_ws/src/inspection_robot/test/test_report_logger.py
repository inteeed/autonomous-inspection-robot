"""Unit tests for ReportLogger observation deduplication and flush output.

Stubs out ROS2 so these run with plain pytest (no ROS2 installation required).
"""
import json
import os
import sys
import time
from unittest.mock import MagicMock

# --- Minimal ROS2 stubs ------------------------------------------------------
import types as _types

class _Node:
    """Stand-in for rclpy.node.Node — provides just enough interface."""
    def __init__(self, *args, **kwargs):
        pass

    def declare_parameter(self, name, default=None):
        pass

    def get_parameter(self, name):
        return MagicMock()

    def create_subscription(self, *args, **kwargs):
        pass

    def create_publisher(self, *args, **kwargs):
        return MagicMock()

    def get_logger(self):
        return MagicMock()

    def destroy_node(self):
        pass


_rclpy_node_mod = _types.ModuleType('rclpy.node')
_rclpy_node_mod.Node = _Node

for _mod, _stub in {
    'rclpy': MagicMock(),
    'std_msgs': MagicMock(),
    'std_msgs.msg': MagicMock(),
}.items():
    sys.modules.setdefault(_mod, _stub)

sys.modules['rclpy.node'] = _rclpy_node_mod
# -----------------------------------------------------------------------------

from inspection_robot.report_logger import ReportLogger  # noqa: E402


def _make_logger(output_dir, expected_ids=None):
    """Build a ReportLogger instance with minimal state, bypassing ROS __init__."""
    node = ReportLogger.__new__(ReportLogger)
    node.run_id = 'run_test'
    node.output_dir = os.path.join(output_dir, 'run_test')
    os.makedirs(node.output_dir, exist_ok=True)
    node.expected = expected_ids if expected_ids is not None else [0, 1, 2, 3, 4]
    node.observations = {}
    node.frames_processed = 0
    node.start_wall = time.time()
    node.flushed = False
    return node


def _det_msg(marker_id, distance=1.0, robot_pose=None):
    """Build a fake detection JSON message."""
    payload = {
        'stamp_sec': 100,
        'stamp_nsec': 0,
        'frame_id': 'camera',
        'robot_pose': robot_pose or {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'detections': [{
            'id': marker_id,
            'tvec': [distance, 0.0, 0.0],
            'rvec': [0.0, 0.0, 0.0],
            'distance_m': distance,
        }],
    }
    msg = MagicMock()
    msg.data = json.dumps(payload)
    return msg


class TestDeduplication:
    def test_first_sighting_creates_entry(self, tmp_path):
        logger = _make_logger(str(tmp_path))
        logger._on_detection(_det_msg(0, distance=2.0))
        assert 0 in logger.observations
        assert logger.observations[0]['sightings'] == 1
        assert logger.observations[0]['best_distance_m'] == 2.0

    def test_closer_sighting_updates_best_distance(self, tmp_path):
        logger = _make_logger(str(tmp_path))
        logger._on_detection(_det_msg(1, distance=3.0))
        logger._on_detection(_det_msg(1, distance=1.5))
        assert logger.observations[1]['best_distance_m'] == 1.5
        assert logger.observations[1]['sightings'] == 2

    def test_farther_sighting_does_not_replace_best(self, tmp_path):
        logger = _make_logger(str(tmp_path))
        logger._on_detection(_det_msg(2, distance=1.0))
        logger._on_detection(_det_msg(2, distance=5.0))
        assert logger.observations[2]['best_distance_m'] == 1.0

    def test_multiple_markers_in_one_frame(self, tmp_path):
        logger = _make_logger(str(tmp_path))
        payload = {
            'stamp_sec': 1, 'stamp_nsec': 0, 'frame_id': 'cam',
            'robot_pose': None,
            'detections': [
                {'id': 0, 'distance_m': 1.0},
                {'id': 1, 'distance_m': 2.0},
            ],
        }
        msg = MagicMock()
        msg.data = json.dumps(payload)
        logger._on_detection(msg)
        assert 0 in logger.observations
        assert 1 in logger.observations

    def test_frames_processed_increments(self, tmp_path):
        logger = _make_logger(str(tmp_path))
        logger._on_detection(_det_msg(0))
        logger._on_detection(_det_msg(1))
        assert logger.frames_processed == 2

    def test_invalid_json_is_ignored(self, tmp_path):
        logger = _make_logger(str(tmp_path))
        msg = MagicMock()
        msg.data = 'not valid json{'
        logger._on_detection(msg)  # must not raise
        assert logger.frames_processed == 0


class TestFlush:
    def test_json_report_written(self, tmp_path):
        logger = _make_logger(str(tmp_path), expected_ids=[0, 1])
        logger._on_detection(_det_msg(0, distance=1.0))
        logger.flush()
        json_path = os.path.join(logger.output_dir, 'report.json')
        assert os.path.exists(json_path)
        with open(json_path) as f:
            report = json.load(f)
        assert report['run_id'] == 'run_test'
        assert 0 in report['detected_markers']
        assert 1 in report['missing_markers']
        assert report['frames_processed'] == 1

    def test_csv_report_written(self, tmp_path):
        logger = _make_logger(str(tmp_path), expected_ids=[0])
        logger._on_detection(_det_msg(0, distance=1.5))
        logger.flush()
        csv_path = os.path.join(logger.output_dir, 'report.csv')
        assert os.path.exists(csv_path)
        with open(csv_path) as f:
            content = f.read()
        assert 'marker_id' in content
        assert 'OK' in content

    def test_flush_is_idempotent(self, tmp_path):
        logger = _make_logger(str(tmp_path))
        logger.flush()
        logger.flush()  # second call must be a no-op
        json_path = os.path.join(logger.output_dir, 'report.json')
        assert os.path.exists(json_path)

    def test_unexpected_marker_flagged(self, tmp_path):
        logger = _make_logger(str(tmp_path), expected_ids=[0])
        logger._on_detection(_det_msg(99, distance=2.0))
        logger.flush()
        with open(os.path.join(logger.output_dir, 'report.json')) as f:
            report = json.load(f)
        assert 99 in report['unexpected_markers']

    def test_all_missing_when_no_detections(self, tmp_path):
        logger = _make_logger(str(tmp_path), expected_ids=[0, 1, 2])
        logger.flush()
        with open(os.path.join(logger.output_dir, 'report.json')) as f:
            report = json.load(f)
        assert report['detected_markers'] == []
        assert set(report['missing_markers']) == {0, 1, 2}
