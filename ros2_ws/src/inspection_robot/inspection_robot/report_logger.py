"""Report logger.

Aggregates JSON detection messages from /inspection/detections, deduplicates
per-marker observations (keeps the closest distance and the first-seen
timestamp), and writes a structured report when /inspection/run_done arrives
True or on shutdown (Ctrl-C).

Output (per run, under output_dir/<run_id>/):
  - report.json : full structured record
  - report.csv  : flat per-marker rows for spreadsheets
"""
from __future__ import annotations

import csv
import json
import os
import time
from datetime import datetime
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String


class ReportLogger(Node):
    def __init__(self):
        super().__init__('report_logger')
        self.declare_parameter('detections_topic', '/inspection/detections')
        self.declare_parameter('done_topic', '/inspection/run_done')
        self.declare_parameter('output_dir', os.path.expanduser('~/inspection_reports'))
        self.declare_parameter('expected_marker_ids', [0, 1, 2, 3, 4])

        self.run_id = datetime.now().strftime('run_%Y%m%d_%H%M%S')
        self.output_dir = os.path.join(
            self.get_parameter('output_dir').value, self.run_id)
        os.makedirs(self.output_dir, exist_ok=True)

        self.expected = list(self.get_parameter('expected_marker_ids').value)
        self.observations: Dict[int, dict] = {}
        self.frames_processed = 0
        self.start_wall = time.time()
        self.flushed = False

        self.create_subscription(
            String, self.get_parameter('detections_topic').value,
            self._on_detection, 10)
        self.create_subscription(
            Bool, self.get_parameter('done_topic').value, self._on_done, 1)

        self.get_logger().info(f'report_logger writing to {self.output_dir}')

    def _on_detection(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.frames_processed += 1
        stamp = payload.get('stamp_sec', 0) + payload.get('stamp_nsec', 0) * 1e-9
        for det in payload.get('detections', []):
            mid = int(det['id'])
            distance = det.get('distance_m')
            existing = self.observations.get(mid)
            if existing is None:
                self.observations[mid] = {
                    'id': mid,
                    'first_seen_stamp': stamp,
                    'first_seen_wall': time.time(),
                    'best_distance_m': distance,
                    'best_tvec': det.get('tvec'),
                    'best_rvec': det.get('rvec'),
                    'best_robot_pose': payload.get('robot_pose'),
                    'sightings': 1,
                }
            else:
                existing['sightings'] += 1
                if (distance is not None and
                        (existing['best_distance_m'] is None or
                         distance < existing['best_distance_m'])):
                    existing['best_distance_m'] = distance
                    existing['best_tvec'] = det.get('tvec')
                    existing['best_rvec'] = det.get('rvec')
                    existing['best_robot_pose'] = payload.get('robot_pose')

    def _on_done(self, msg: Bool):
        if msg.data:
            self.flush()

    def flush(self):
        if self.flushed:
            return
        self.flushed = True
        seen = set(self.observations.keys())
        expected = set(int(m) for m in self.expected)
        report = {
            'run_id': self.run_id,
            'generated_at': datetime.now().isoformat(timespec='seconds'),
            'duration_s': round(time.time() - self.start_wall, 2),
            'frames_processed': self.frames_processed,
            'expected_markers': sorted(expected),
            'detected_markers': sorted(seen),
            'missing_markers': sorted(expected - seen),
            'unexpected_markers': sorted(seen - expected),
            'observations': sorted(self.observations.values(), key=lambda o: o['id']),
        }
        json_path = os.path.join(self.output_dir, 'report.json')
        with open(json_path, 'w') as f:
            json.dump(report, f, indent=2)

        csv_path = os.path.join(self.output_dir, 'report.csv')
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['marker_id', 'sightings', 'first_seen_wall',
                        'best_distance_m', 'tvec_x', 'tvec_y', 'tvec_z',
                        'robot_x', 'robot_y', 'robot_yaw', 'status'])
            for mid in sorted(expected | seen):
                obs = self.observations.get(mid)
                if obs is None:
                    w.writerow([mid, 0, '', '', '', '', '', '', '', '', 'MISSING'])
                    continue
                tv = obs.get('best_tvec') or [None, None, None]
                rp = obs.get('best_robot_pose') or {}
                status = 'OK' if mid in expected else 'UNEXPECTED'
                w.writerow([
                    mid, obs['sightings'], obs['first_seen_wall'],
                    obs.get('best_distance_m'),
                    tv[0], tv[1], tv[2],
                    rp.get('x'), rp.get('y'), rp.get('yaw'),
                    status,
                ])
        self.get_logger().info(
            f'Report written: {json_path} ({len(seen)}/{len(expected)} markers detected)')


def main(args=None):
    rclpy.init(args=args)
    node = ReportLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.flush()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
