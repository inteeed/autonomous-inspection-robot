"""Nav2-backed waypoint inspection sequencer."""
from __future__ import annotations

import math
import json
from typing import List, Optional, Tuple

import yaml

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String

from inspection_robot.waypoint_state_machine import should_skip_label


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class Nav2WaypointFollower(Node):
    def __init__(self):
        super().__init__('nav2_waypoint_follower')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('dwell_seconds', 4.0)
        self.declare_parameter('autostart', True)
        self.declare_parameter('run_request_topic', '/inspection/run_request')
        self.declare_parameter('skip_markers_topic', '/inspection/skip_markers')

        self.waypoints: List[Tuple[float, float, float, str]] = self._load_waypoints(
            str(self.get_parameter('waypoints_file').value))
        self.idx = 0
        self.active = bool(self.get_parameter('autostart').value)
        self.goal_in_flight = False
        self.dwell_t0 = None
        self.finished = False
        self.skip_marker_ids = set()

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/inspection/status', 10)
        self.done_pub = self.create_publisher(Bool, '/inspection/run_done', 1)
        self.create_subscription(
            Bool,
            str(self.get_parameter('run_request_topic').value),
            self._on_run_request,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('skip_markers_topic').value),
            self._on_skip_markers,
            10,
        )
        self.timer = self.create_timer(0.5, self._tick)

        mode = 'autostart' if self.active else 'waiting for run request'
        self.get_logger().info(
            f'Nav2 waypoint follower loaded {len(self.waypoints)} waypoints; {mode}.')

    @staticmethod
    def _load_waypoints(path: str):
        if not path:
            return []
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        return [
            (
                float(w['x']),
                float(w['y']),
                float(w.get('yaw', 0.0)),
                str(w.get('label', '')),
            )
            for w in data.get('waypoints', [])
        ]

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _on_run_request(self, msg: Bool):
        if not msg.data:
            return
        self.idx = 0
        self.active = True
        self.finished = False
        self.goal_in_flight = False
        self.dwell_t0 = None
        self._publish_status('nav2_run_started')

    def _on_skip_markers(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Ignoring malformed skip marker payload.')
            return
        self.skip_marker_ids = {int(v) for v in payload.get('marker_ids', [])}
        self._publish_status(f'skip_markers:{sorted(self.skip_marker_ids)}')

    def _tick(self):
        if not self.active or self.finished or self.goal_in_flight:
            return
        if not self.waypoints:
            self.get_logger().error('No waypoints configured for Nav2 follower.')
            self._finish()
            return
        if self.dwell_t0 is not None:
            elapsed = (self.get_clock().now() - self.dwell_t0).nanoseconds / 1e9
            if elapsed < float(self.get_parameter('dwell_seconds').value):
                return
            self.dwell_t0 = None
            self.idx += 1
        if self.idx >= len(self.waypoints):
            self._finish()
            return
        if self._should_skip_current_waypoint():
            _, _, _, label = self.waypoints[self.idx]
            self._publish_status(f'skipping@{self.idx}:{label}')
            self.idx += 1
            return
        self._send_current_goal()

    def _should_skip_current_waypoint(self) -> bool:
        _, _, _, label = self.waypoints[self.idx]
        return should_skip_label(label, self.skip_marker_ids)

    def _send_current_goal(self):
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('Waiting for Nav2 navigate_to_pose action server...')
            return

        x, y, yaw, label = self.waypoints[self.idx]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = str(self.get_parameter('global_frame').value)
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = yaw_to_quaternion(yaw)

        self.goal_in_flight = True
        self._publish_status(f'navigating@{self.idx}:{label}')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_in_flight = False
            self._publish_status(f'nav2_goal_rejected@{self.idx}')
            self.get_logger().warn(f'Nav2 rejected waypoint {self.idx}; skipping.')
            self.idx += 1
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        result = future.result()
        self.goal_in_flight = False
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            _, _, _, label = self.waypoints[self.idx]
            self.dwell_t0 = self.get_clock().now()
            self._publish_status(f'dwelling@{self.idx}:{label}')
        else:
            self._publish_status(f'nav2_goal_failed@{self.idx}:status={result.status}')
            self.get_logger().warn(
                f'Waypoint {self.idx} failed with Nav2 status {result.status}; continuing.')
            self.idx += 1

    def _finish(self):
        self.finished = True
        self.active = False
        self.done_pub.publish(Bool(data=True))
        self._publish_status('all_waypoints_complete')
        self.get_logger().info('All Nav2 waypoints complete.')


def main(args=None):
    rclpy.init(args=args)
    node = Nav2WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
