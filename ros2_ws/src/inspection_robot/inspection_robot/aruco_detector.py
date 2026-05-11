"""ArUco detector node.

Subscribes to a Gazebo camera image stream, detects ArUco markers using the
OpenCV 4.2 legacy aruco API (Dictionary_get / DetectorParameters_create /
detectMarkers), and publishes one std_msgs/String per frame containing a JSON
list of detections. The report_logger node is the consumer.

The JSON message shape per frame (one message per frame that contains >=1 marker):
    {
      "stamp_sec":    <int>,
      "stamp_nsec":   <int>,
      "frame_id":     <str>,
      "robot_pose":   {"x":..,"y":..,"yaw":..} | null,
      "detections":   [
          {"id": <int>, "tvec": [x,y,z], "rvec": [rx,ry,rz], "distance_m": <float>}
      ]
    }

Pose estimation uses CameraInfo intrinsics when available; otherwise tvec/rvec
are omitted and only IDs are reported. Robot pose is sampled from /odom.
"""
from __future__ import annotations

import json
import math
from typing import Optional

import cv2
import cv2.aruco as aruco
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('detections_topic', '/inspection/detections')
        self.declare_parameter('marker_size_m', 0.4)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('min_consecutive_detections', 2)
        self.declare_parameter('calibration_file', '')

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.last_odom: Optional[Odometry] = None
        self._consecutive: dict = {}  # marker_id -> consecutive frame count

        dict_name = self.get_parameter('aruco_dictionary').value
        dict_id = getattr(aruco, dict_name, aruco.DICT_4X4_50)
        self.dictionary = aruco.Dictionary_get(dict_id)
        self.params = aruco.DetectorParameters_create()
        self.marker_size = float(self.get_parameter('marker_size_m').value)

        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        det_topic = self.get_parameter('detections_topic').value

        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)

        self.det_pub = self.create_publisher(String, det_topic, 10)
        if self.get_parameter('publish_annotated').value:
            self.annot_pub = self.create_publisher(Image, '/inspection/annotated', 1)
        else:
            self.annot_pub = None

        cal_file = self.get_parameter('calibration_file').value
        if cal_file:
            self._load_calibration_file(cal_file)

        self.get_logger().info(
            f'aruco_detector listening on {image_topic} (info={info_topic}, odom={odom_topic}) '
            f'-> publishing detections to {det_topic}')

    def _load_calibration_file(self, path: str):
        """Load camera intrinsics from a YAML calibration file as a startup fallback."""
        try:
            import yaml
            with open(path, 'r') as f:
                cal = yaml.safe_load(f)
            k_data = cal.get('camera_matrix', {}).get('data', [])
            d_data = cal.get('distortion_coefficients', {}).get('data', [])
            if len(k_data) == 9:
                self.camera_matrix = np.array(k_data, dtype=np.float64).reshape(3, 3)
                self.dist_coeffs = np.array(d_data, dtype=np.float64) if d_data else np.zeros(5)
                self.get_logger().info(f'Loaded camera calibration from {path}')
            else:
                self.get_logger().warn(f'Calibration file {path} has no valid camera_matrix')
        except Exception as e:
            self.get_logger().warn(f'Failed to load calibration file {path}: {e}')

    def _on_info(self, msg: CameraInfo):
        # Live CameraInfo always takes precedence over the file fallback.
        k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        d = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)
        first = self.camera_matrix is None
        self.camera_matrix = k
        self.dist_coeffs = d
        if first:
            self.get_logger().info('Got CameraInfo, pose estimation enabled.')

    def _on_odom(self, msg: Odometry):
        self.last_odom = msg

    def _on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.dictionary, parameters=self.params)

        current_ids = set(ids.flatten().tolist()) if ids is not None and len(ids) > 0 else set()
        # Reset streak for any marker that disappeared this frame
        for mid in list(self._consecutive.keys()):
            if mid not in current_ids:
                del self._consecutive[mid]

        # Always publish the annotated frame so operators see raw detections in RViz
        if self.annot_pub is not None and ids is not None and len(ids) > 0:
            annotated = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
            try:
                self.annot_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
            except Exception:
                pass

        if not current_ids:
            return

        rvecs = tvecs = None
        if self.camera_matrix is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

        min_consec = int(self.get_parameter('min_consecutive_detections').value)
        detections = []
        for i, marker_id in enumerate(ids.flatten().tolist()):
            self._consecutive[marker_id] = self._consecutive.get(marker_id, 0) + 1
            if self._consecutive[marker_id] < min_consec:
                continue  # not enough consecutive frames yet
            entry = {'id': int(marker_id)}
            if tvecs is not None:
                t = tvecs[i].flatten().tolist()
                r = rvecs[i].flatten().tolist()
                entry['tvec'] = [float(v) for v in t]
                entry['rvec'] = [float(v) for v in r]
                entry['distance_m'] = float(np.linalg.norm(tvecs[i]))
            detections.append(entry)

        if not detections:
            return

        robot_pose = None
        if self.last_odom is not None:
            p = self.last_odom.pose.pose
            robot_pose = {
                'x': float(p.position.x),
                'y': float(p.position.y),
                'yaw': float(quat_to_yaw(p.orientation.x, p.orientation.y,
                                         p.orientation.z, p.orientation.w)),
            }

        payload = {
            'stamp_sec': int(msg.header.stamp.sec),
            'stamp_nsec': int(msg.header.stamp.nanosec),
            'frame_id': msg.header.frame_id,
            'robot_pose': robot_pose,
            'detections': detections,
        }
        out = String()
        out.data = json.dumps(payload)
        self.det_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
