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


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('position_tolerance', 0.15)
        self.declare_parameter('yaw_tolerance', 0.10)
        self.declare_parameter('dwell_seconds', 4.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('laser_topic', '/scan')
        self.declare_parameter('obstacle_distance_m', 0.40)

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
            LaserScan, self.get_parameter('laser_topic').value,
            self._on_scan, qos_profile_sensor_data)

        # Internal state
        self.current_pose = None  # (x, y, yaw)
        self.last_scan: Optional[LaserScan] = None
        self.idx = 0
        self.phase = 'turn'        # 'turn' -> 'drive' -> 'align' -> 'dwell'
        self.dwell_t0 = None
        self.run_finished = False

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
        self.last_scan = msg

    def _obstacle_steer(self) -> float:
        """Check the forward laser arc for obstacles.

        Returns a signed angular velocity (+left / -right) when the forward
        path is blocked, or 0.0 when it is clear.
        """
        if self.last_scan is None:
            return 0.0
        scan = self.last_scan
        threshold = float(self.get_parameter('obstacle_distance_m').value)
        fwd_half = 0.35  # ±20° forward detection window

        fwd_min = float('inf')
        left_min = float('inf')
        right_min = float('inf')
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min <= r <= scan.range_max):
                continue
            a = scan.angle_min + i * scan.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))  # normalize to [-π, π]
            if abs(a) <= fwd_half:
                fwd_min = min(fwd_min, r)
            if 0.0 < a <= 1.2:       # forward-left sector
                left_min = min(left_min, r)
            elif -1.2 <= a < 0.0:    # forward-right sector
                right_min = min(right_min, r)

        if fwd_min >= threshold:
            return 0.0  # path clear

        v_ang = float(self.get_parameter('angular_speed').value)
        # Steer toward the side with more clearance
        return v_ang if left_min >= right_min else -v_ang

    def _publish_status(self, text: str):
        m = String(); m.data = text
        self.status_pub.publish(m)

    def _stop(self):
        self.cmd_pub.publish(Twist())

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
        x, y, yaw = self.current_pose
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        heading_to_goal = math.atan2(dy, dx)

        v_lin = float(self.get_parameter('linear_speed').value)
        v_ang = float(self.get_parameter('angular_speed').value)
        pos_tol = float(self.get_parameter('position_tolerance').value)
        yaw_tol = float(self.get_parameter('yaw_tolerance').value)
        dwell_s = float(self.get_parameter('dwell_seconds').value)

        cmd = Twist()
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
                avoid = self._obstacle_steer()
                if avoid != 0.0:
                    cmd.linear.x = v_lin * 0.3   # slow while sidestepping
                    cmd.angular.z = avoid
                else:
                    err = angle_diff(heading_to_goal, yaw)
                    cmd.linear.x = v_lin * max(0.0, math.cos(err))
                    cmd.angular.z = max(-v_ang, min(v_ang, 1.5 * err))
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
