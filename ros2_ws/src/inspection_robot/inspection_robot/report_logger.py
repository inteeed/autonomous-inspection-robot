"""Report logger.

Aggregates JSON detection messages from /inspection/detections, deduplicates
per-marker observations (keeps the closest distance and the first-seen
timestamp), and writes a structured report when /inspection/run_done arrives
True or on shutdown (Ctrl-C).

Output (per run, under output_dir/<run_id>/):
  - report.json   : full structured record (includes cross-run history)
  - report.csv    : flat per-marker rows for spreadsheets
  - report.pdf    : human-readable PDF with embedded anomaly snapshots

Cross-run history is maintained in output_dir/history.json.
"""
from __future__ import annotations

import csv
import json
import os
import time
from datetime import datetime
from typing import Dict

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from inspection_robot import report_utils


class ReportLogger(Node):
    def __init__(self):
        super().__init__('report_logger')
        self.declare_parameter('detections_topic', '/inspection/detections')
        self.declare_parameter('anomalies_topic', '/inspection/anomalies')
        self.declare_parameter('done_topic', '/inspection/run_done')
        self.declare_parameter('output_dir', os.path.expanduser('~/inspection_reports'))
        self.declare_parameter('expected_marker_ids', [0, 1, 2, 3, 4])
        self.declare_parameter('write_pdf', True)

        self.run_id = datetime.now().strftime('run_%Y%m%d_%H%M%S')
        self.output_dir = os.path.join(
            self.get_parameter('output_dir').value, self.run_id)
        os.makedirs(self.output_dir, exist_ok=True)

        self.expected = list(self.get_parameter('expected_marker_ids').value)
        self.observations: Dict[int, dict] = {}
        self.anomalies: Dict[int, dict] = {}
        self.frames_processed = 0
        self.start_wall = time.time()
        self.flushed = False

        self.create_subscription(
            String, self.get_parameter('detections_topic').value,
            self._on_detection, 10)
        self.create_subscription(
            String, self.get_parameter('anomalies_topic').value,
            self._on_anomaly, 10)
        self.create_subscription(
            Bool, self.get_parameter('done_topic').value, self._on_done, 1)

        self._history_path = os.path.join(
            str(self.get_parameter('output_dir').value), 'history.json')
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
            report_utils.merge_detection_observation(
                self.observations, mid, det, payload.get('robot_pose'), stamp, time.time())

    def _on_anomaly(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        for event in payload.get('anomalies', []):
            mid = int(event['id'])
            existing = self.anomalies.get(mid)
            if existing is None or event.get('status') == 'ANOMALY':
                self.anomalies[mid] = {
                    'id': mid,
                    'status': event.get('status', 'UNKNOWN'),
                    'types': event.get('types', []),
                    'scores': event.get('scores', {}),
                    'snapshot_path': event.get('snapshot_path'),
                    'stamp_wall': payload.get('stamp_wall', time.time()),
                }

    def _on_done(self, msg: Bool):
        if msg.data:
            self.flush()

    def _load_history(self) -> dict:
        try:
            with open(self._history_path, 'r') as f:
                return json.load(f)
        except (OSError, json.JSONDecodeError):
            return {'runs': [], 'markers': {}}

    def _save_history(self, history: dict):
        try:
            with open(self._history_path, 'w') as f:
                json.dump(history, f, indent=2)
        except OSError as e:
            self.get_logger().warn(f'Could not write history: {e}')

    def _update_history(self, history: dict, seen: set, anomalies: dict):
        history.setdefault('runs', []).append(self.run_id)
        history.setdefault('markers', {})
        for mid in set(int(m) for m in self.expected) | seen:
            key = str(mid)
            entry = history['markers'].setdefault(key, {
                'total_runs': 0,
                'anomaly_runs': 0,
                'last_status': 'MISSING',
                'consecutive_anomalies': 0,
                'last_run': '',
            })
            entry['total_runs'] += 1
            entry['last_run'] = self.run_id
            if mid not in seen:
                entry['last_status'] = 'MISSING'
                entry['consecutive_anomalies'] = 0
            elif anomalies.get(mid, {}).get('status') == 'ANOMALY':
                entry['anomaly_runs'] += 1
                entry['consecutive_anomalies'] += 1
                entry['last_status'] = 'ANOMALY'
            else:
                entry['consecutive_anomalies'] = 0
                entry['last_status'] = 'PASS'
        return history

    def flush(self):
        if self.flushed:
            return
        self.flushed = True
        seen = set(self.observations.keys())
        expected = set(int(m) for m in self.expected)
        history = self._load_history()
        history = self._update_history(history, seen, self.anomalies)
        self._save_history(history)
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
            'anomalies': sorted(self.anomalies.values(), key=lambda o: o['id']),
            'history': history.get('markers', {}),
        }
        json_path = os.path.join(self.output_dir, 'report.json')
        with open(json_path, 'w') as f:
            json.dump(report, f, indent=2)

        csv_path = os.path.join(self.output_dir, 'report.csv')
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['marker_id', 'sightings', 'first_seen_wall',
                        'best_distance_m', 'tvec_x', 'tvec_y', 'tvec_z',
                        'robot_x', 'robot_y', 'robot_yaw',
                        'inspection_status', 'anomaly_types', 'snapshot_path'])
            for mid in sorted(expected | seen):
                obs = self.observations.get(mid)
                anomaly = self.anomalies.get(mid, {})
                if obs is None:
                    w.writerow([mid, 0, '', '', '', '', '', '', '', '',
                                'MISSING', '', ''])
                    continue
                tv = obs.get('best_tvec') or [None, None, None]
                rp = obs.get('best_robot_pose') or {}
                if mid not in expected:
                    status = 'UNEXPECTED'
                elif anomaly.get('status') == 'ANOMALY':
                    status = 'FAIL'
                else:
                    status = 'PASS'
                w.writerow([
                    mid, obs['sightings'], obs['first_seen_wall'],
                    obs.get('best_distance_m'),
                    tv[0], tv[1], tv[2],
                    rp.get('x'), rp.get('y'), rp.get('yaw'),
                    status,
                    ';'.join(anomaly.get('types', [])),
                    anomaly.get('snapshot_path', ''),
                ])
        if bool(self.get_parameter('write_pdf').value):
            pdf_path = os.path.join(self.output_dir, 'report.pdf')
            report_utils.write_simple_pdf(pdf_path, report)
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
