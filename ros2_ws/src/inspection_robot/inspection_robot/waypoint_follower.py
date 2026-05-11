"""Scripted waypoint follower.

Reads a list of (x, y, yaw) waypoints from a YAML config and drives the robot
through them in order using odometry feedback and a simple two-stage P
controller (turn-toward-goal, then drive-forward, then face-final-yaw). At
each waypoint the robot pauses for `dwell_seconds` so the aruco_detector has
time to observe nearby markers. After the final waypoint the node publishes
std_msgs/Bool=True on /inspection/run_done so the report_logger flushes.

This is intentionally not Nav2 — no map, no costmap, no planner. The world is
open enough that going straight between waypoints works for the demo.
"""
from __future__ import annotations

import json
import math
from typing import List, Optional, Tuple

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from inspection_robot.waypoint_state_machine import choose_avoid_direction, should_skip_label


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('position_tolerance', 0.15)
        self.declare_parameter('yaw_tolerance', 0.10)
        self.declare_parameter('dwell_seconds', 4.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('autostart', True)
        self.declare_parameter('run_request_topic', '/inspection/run_request')
        self.declare_parameter('skip_markers_topic', '/inspection/skip_markers')
        self.declare_parameter('enable_obstacle_avoidance', True)
        self.declare_parameter('obstacle_distance_threshold', 0.65)
        self.declare_parameter('clear_distance_threshold', 0.90)
        self.declare_parameter('front_sector_deg', 25.0)
        self.declare_parameter('side_sector_deg', 70.0)
        self.declare_parameter('avoid_heading_offset_rad', 0.95)
        self.declare_parameter('avoid_heading_tolerance', 0.30)
        self.declare_parameter('avoid_linear_speed', 0.10)
        self.declare_parameter('avoid_angular_speed', 0.8)
        self.declare_parameter('scan_timeout_s', 0.75)
        self.declare_parameter('stuck_timeout_s', 8.0)
        self.declare_parameter('stuck_min_progress_m', 0.05)
        self.declare_parameter('recovery_backup_speed', -0.12)
        self.declare_parameter('recovery_backup_duration_s', 1.8)

        path = self.get_parameter('waypoints_file').value
        self.waypoints: List[Tuple[float, float, float, str]] = self._load_waypoints(path)
        if not self.waypoints:
            self.get_logger().error(f'No waypoints loaded from {path!r}')
        else:
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} waypoints from {path}')

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.status_pub = self.create_publisher(String, '/inspection/status', 10)
        self.done_pub = self.create_publisher(Bool, '/inspection/run_done', 1)

        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Bool, self.get_parameter('run_request_topic').value,
            self._on_run_request, 10)
        self.create_subscription(
            String, self.get_parameter('skip_markers_topic').value,
            self._on_skip_markers, 10)

        # Internal state
        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.latest_scan: Optional[LaserScan] = None
        self.last_scan_time = None
        self.idx = 0
        self.phase = 'turn'        # 'turn' | 'drive' | 'avoid' | 'align' | 'dwell' | 'recovery'
        self.dwell_t0 = None
        self.run_finished = not bool(self.get_parameter('autostart').value)
        self.avoid_direction = 1
        self.skip_marker_ids = set()
        # Stuck detection
        self._stuck_ref_pos: Optional[Tuple[float, float]] = None
        self._stuck_ref_time: Optional[float] = None
        self._recovery_t0: Optional[float] = None
        self._recovery_attempts = 0

        rate = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self._tick)

    @staticmethod
    def _load_waypoints(path: str):
        if not path:
            return []
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        out = []
        for w in data.get('waypoints', []):
            out.append((
                float(w['x']),
                float(w['y']),
                float(w.get('yaw', 0.0)),
                str(w.get('label', '')),
            ))
        return out

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.current_pose = (
            float(p.position.x),
            float(p.position.y),
            float(quat_to_yaw(p.orientation.x, p.orientation.y,
                              p.orientation.z, p.orientation.w)),
        )

    def _on_scan(self, msg: LaserScan):
        self.latest_scan = msg
        self.last_scan_time = self.get_clock().now()

    def _on_run_request(self, msg: Bool):
        if not msg.data:
            return
        self.idx = 0
        self.phase = 'turn'
        self.dwell_t0 = None
        self.run_finished = False
        self._stuck_ref_pos = None
        self._stuck_ref_time = None
        self._recovery_attempts = 0
        self._publish_status('pid_run_started')

    def _on_skip_markers(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Ignoring malformed skip marker payload.')
            return
        self.skip_marker_ids = {int(v) for v in payload.get('marker_ids', [])}
        self._publish_status(f'skip_markers:{sorted(self.skip_marker_ids)}')

    def _publish_status(self, text: str):
        m = String(); m.data = text
        self.status_pub.publish(m)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _scan_is_fresh(self) -> bool:
        if self.latest_scan is None or self.last_scan_time is None:
            return False
        age_s = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        return age_s <= float(self.get_parameter('scan_timeout_s').value)

    def _sector_min_range(self, start_deg: float, end_deg: float) -> float:
        if not self._scan_is_fresh():
            return math.inf
        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)
        best = math.inf
        angle = self.latest_scan.angle_min
        for reading in self.latest_scan.ranges:
            if start_rad <= angle <= end_rad:
                if (math.isfinite(reading) and
                        self.latest_scan.range_min < reading <= self.latest_scan.range_max):
                    best = min(best, float(reading))
            angle += self.latest_scan.angle_increment
        return best

    def _scan_metrics(self):
        front_deg = float(self.get_parameter('front_sector_deg').value)
        side_deg = float(self.get_parameter('side_sector_deg').value)
        return {
            'front': self._sector_min_range(-front_deg, front_deg),
            'left': self._sector_min_range(10.0, side_deg),
            'right': self._sector_min_range(-side_deg, -10.0),
        }

    def _path_blocked(self, metrics) -> bool:
        if not bool(self.get_parameter('enable_obstacle_avoidance').value):
            return False
        return metrics['front'] < float(self.get_parameter('obstacle_distance_threshold').value)

    def _enter_avoidance(self, label: str, metrics):
        self.avoid_direction = choose_avoid_direction(metrics['left'], metrics['right'])
        self.phase = 'avoid'
        self._reset_stuck_ref()
        direction = 'left' if self.avoid_direction > 0 else 'right'
        self._publish_status(f'avoiding_obstacle@{self.idx}:{label}:{direction}')
        self.get_logger().info(
            f'Obstacle blocking waypoint {self.idx} ({label}); rerouting {direction}.')

    def _reset_stuck_ref(self):
        if self.current_pose is not None:
            self._stuck_ref_pos = (self.current_pose[0], self.current_pose[1])
        self._stuck_ref_time = self.get_clock().now()

    def _check_stuck(self, label: str) -> bool:
        """Return True and enter recovery if the robot hasn't moved enough."""
        if self.current_pose is None:
            return False
        now = self.get_clock().now()
        timeout = float(self.get_parameter('stuck_timeout_s').value)
        min_progress = float(self.get_parameter('stuck_min_progress_m').value)

        if self._stuck_ref_pos is None or self._stuck_ref_time is None:
            self._reset_stuck_ref()
            return False

        elapsed = (now - self._stuck_ref_time).nanoseconds / 1e9
        dx = self.current_pose[0] - self._stuck_ref_pos[0]
        dy = self.current_pose[1] - self._stuck_ref_pos[1]
        progress = math.hypot(dx, dy)

        if progress >= min_progress:
            self._reset_stuck_ref()
            return False

        if elapsed >= timeout:
            self._recovery_attempts += 1
            self.phase = 'recovery'
            self._recovery_t0 = now
            self._publish_status(f'stuck_recovery@{self.idx}:{label}:attempt{self._recovery_attempts}')
            self.get_logger().warn(
                f'Stuck at waypoint {self.idx} ({label}) — backing up '
                f'(attempt {self._recovery_attempts}).')
            self._reset_stuck_ref()
            return True
        return False

    def _tick(self):
        if self.run_finished or self.current_pose is None:
            return
        if self.idx >= len(self.waypoints):
            if not self.run_finished:
                self._stop()
                self.done_pub.publish(Bool(data=True))
                self._publish_status('all_waypoints_complete')
                self.get_logger().info('All waypoints complete.')
                self.run_finished = True
            return

        gx, gy, gyaw, label = self.waypoints[self.idx]
        if self._should_skip_waypoint(label):
            self._publish_status(f'skipping@{self.idx}:{label}')
            self.idx += 1
            self.phase = 'turn'
            self._reset_stuck_ref()
            return
        x, y, yaw = self.current_pose
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        heading_to_goal = math.atan2(dy, dx)

        v_lin = float(self.get_parameter('linear_speed').value)
        v_ang = float(self.get_parameter('angular_speed').value)
        pos_tol = float(self.get_parameter('position_tolerance').value)
        yaw_tol = float(self.get_parameter('yaw_tolerance').value)
        dwell_s = float(self.get_parameter('dwell_seconds').value)
        metrics = self._scan_metrics()

        cmd = Twist()
        if self.phase == 'drive' and dist >= pos_tol and self._path_blocked(metrics):
            self._enter_avoidance(label, metrics)
        elif self.phase == 'drive' and dist >= pos_tol:
            self._check_stuck(label)

        if self.phase == 'turn':
            err = angle_diff(heading_to_goal, yaw)
            if abs(err) < yaw_tol or dist < pos_tol:
                self.phase = 'drive'
            else:
                cmd.angular.z = max(-v_ang, min(v_ang, 1.5 * err))
        elif self.phase == 'drive':
            if dist < pos_tol:
                self.phase = 'align'
            else:
                err = angle_diff(heading_to_goal, yaw)
                cmd.linear.x = v_lin * max(0.0, math.cos(err))
                cmd.angular.z = max(-v_ang, min(v_ang, 1.5 * err))
        elif self.phase == 'avoid':
            avoid_ang = float(self.get_parameter('avoid_angular_speed').value)
            avoid_lin = float(self.get_parameter('avoid_linear_speed').value)
            heading_offset = float(self.get_parameter('avoid_heading_offset_rad').value)
            resume_tol = float(self.get_parameter('avoid_heading_tolerance').value)
            obstacle_threshold = float(
                self.get_parameter('obstacle_distance_threshold').value)
            clear_threshold = float(
                self.get_parameter('clear_distance_threshold').value)

            avoid_heading = heading_to_goal + self.avoid_direction * heading_offset
            avoid_err = angle_diff(avoid_heading, yaw)
            goal_err = angle_diff(heading_to_goal, yaw)

            if metrics['front'] < obstacle_threshold:
                cmd.angular.z = self.avoid_direction * avoid_ang
            else:
                cmd.linear.x = avoid_lin * max(0.0, math.cos(avoid_err))
                cmd.angular.z = clamp(1.5 * avoid_err, -avoid_ang, avoid_ang)

            if metrics['front'] > clear_threshold and abs(goal_err) < resume_tol:
                self.phase = 'turn'
                self._publish_status(f'resuming_waypoint@{self.idx}:{label}')
        elif self.phase == 'recovery':
            backup_speed = float(self.get_parameter('recovery_backup_speed').value)
            backup_dur = float(self.get_parameter('recovery_backup_duration_s').value)
            elapsed = (self.get_clock().now() - self._recovery_t0).nanoseconds / 1e9
            if elapsed < backup_dur:
                cmd.linear.x = backup_speed
            else:
                self.phase = 'turn'
                self._publish_status(f'recovery_done@{self.idx}:{label}')
        elif self.phase == 'align':
            err = angle_diff(gyaw, yaw)
            if abs(err) < yaw_tol:
                self.phase = 'dwell'
                self.dwell_t0 = self.get_clock().now()
                self._publish_status(f'dwelling@{self.idx}:{label}')
                self.get_logger().info(
                    f'Reached waypoint {self.idx} ({label}) — dwelling {dwell_s:.1f}s')
            else:
                cmd.angular.z = max(-v_ang, min(v_ang, 1.5 * err))
        elif self.phase == 'dwell':
            elapsed = (self.get_clock().now() - self.dwell_t0).nanoseconds / 1e9
            if elapsed >= dwell_s:
                self.idx += 1
                self.phase = 'turn'

        self.cmd_pub.publish(cmd)

    def _should_skip_waypoint(self, label: str) -> bool:
        return should_skip_label(label, self.skip_marker_ids)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
