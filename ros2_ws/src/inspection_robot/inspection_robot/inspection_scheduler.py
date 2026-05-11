"""Time-based inspection run scheduler with nominal-marker skip hints."""
from __future__ import annotations

import json
import re
import time
from typing import Dict, Set

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class InspectionScheduler(Node):
    def __init__(self):
        super().__init__('inspection_scheduler')
        self.declare_parameter('schedule_period_s', 3600.0)
        self.declare_parameter('run_request_topic', '/inspection/run_request')
        self.declare_parameter('skip_markers_topic', '/inspection/skip_markers')
        self.declare_parameter('status_topic', '/inspection/status')
        self.declare_parameter('anomalies_topic', '/inspection/anomalies')
        self.declare_parameter('recent_nominal_ttl_s', 86400.0)
        self.declare_parameter('autostart_first_run', False)

        self.last_nominal_seen: Dict[int, float] = {}
        self.in_progress = False
        now = time.time()
        self.next_run_at = now if bool(self.get_parameter('autostart_first_run').value) else (
            now + float(self.get_parameter('schedule_period_s').value))

        self.run_pub = self.create_publisher(
            Bool, str(self.get_parameter('run_request_topic').value), 10)
        self.skip_pub = self.create_publisher(
            String, str(self.get_parameter('skip_markers_topic').value), 10)
        self.create_subscription(
            String, str(self.get_parameter('status_topic').value), self._on_status, 10)
        self.create_subscription(
            String, str(self.get_parameter('anomalies_topic').value), self._on_anomaly, 10)
        self.timer = self.create_timer(1.0, self._tick)
        self.get_logger().info('inspection_scheduler ready.')

    def _on_status(self, msg: String):
        if msg.data in ('all_waypoints_complete', 'nav2_run_started'):
            self.in_progress = msg.data == 'nav2_run_started'

    def _on_anomaly(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        now = time.time()
        for event in payload.get('anomalies', []):
            if event.get('status') == 'NOMINAL':
                self.last_nominal_seen[int(event['id'])] = now

    def _recent_nominal_markers(self) -> Set[int]:
        ttl = float(self.get_parameter('recent_nominal_ttl_s').value)
        now = time.time()
        return {
            marker_id
            for marker_id, seen_at in self.last_nominal_seen.items()
            if now - seen_at <= ttl
        }

    def _tick(self):
        if self.in_progress or time.time() < self.next_run_at:
            return
        skip_msg = String()
        skip_msg.data = json.dumps({'marker_ids': sorted(self._recent_nominal_markers())})
        self.skip_pub.publish(skip_msg)
        self.run_pub.publish(Bool(data=True))
        self.in_progress = True
        self.next_run_at = time.time() + float(self.get_parameter('schedule_period_s').value)


def marker_id_from_label(label: str):
    match = re.search(r'marker_(\d+)', label)
    return int(match.group(1)) if match else None


def main(args=None):
    rclpy.init(args=args)
    node = InspectionScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
