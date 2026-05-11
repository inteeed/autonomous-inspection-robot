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
import os
from typing import Dict, Optional

import cv2
import cv2.aruco as aruco
import numpy as np
import yaml

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


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


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
        self.declare_parameter('annotated_topic', '/inspection/annotated')
        self.declare_parameter('camera_calibration_file', '')
        self.declare_parameter('use_camera_info_fallback', True)
        self.declare_parameter('min_detection_confidence', 0.6)
        self.declare_parameter('required_consecutive_detections', 2)
        self.declare_parameter('confidence_area_reference', 0.01)

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.camera_source = 'uninitialized'
        self.last_odom: Optional[Odometry] = None
        self.detection_streaks: Dict[int, int] = {}

        dict_name = self.get_parameter('aruco_dictionary').value
        dict_id = getattr(aruco, dict_name, aruco.DICT_4X4_50)
        self.dictionary = aruco.Dictionary_get(dict_id)
        self.params = aruco.DetectorParameters_create()
        if hasattr(aruco, 'CORNER_REFINE_SUBPIX'):
            self.params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.min_confidence = float(self.get_parameter('min_detection_confidence').value)
        self.required_consecutive = max(
            1, int(self.get_parameter('required_consecutive_detections').value))
        self.area_reference = max(
            1e-6, float(self.get_parameter('confidence_area_reference').value))

        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        det_topic = self.get_parameter('detections_topic').value
        annotated_topic = self.get_parameter('annotated_topic').value

        calibration_path = str(self.get_parameter('camera_calibration_file').value)
        if calibration_path:
            self._load_calibration_file(calibration_path)

        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        if (self.camera_matrix is None or
                bool(self.get_parameter('use_camera_info_fallback').value)):
            self.create_subscription(
                CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)

        self.det_pub = self.create_publisher(String, det_topic, 10)
        if self.get_parameter('publish_annotated').value:
            self.annot_pub = self.create_publisher(
                Image, annotated_topic, qos_profile_sensor_data)
        else:
            self.annot_pub = None

        self.get_logger().info(
            f'aruco_detector listening on {image_topic} (info={info_topic}, odom={odom_topic}) '
            f'-> publishing detections to {det_topic}')
        if self.annot_pub is not None:
            self.get_logger().info(f'Annotated image stream enabled on {annotated_topic}')
        if self.camera_matrix is not None:
            self.get_logger().info(f'Using camera calibration from {self.camera_source}')
        else:
            self.get_logger().warn(
                'No camera calibration loaded yet; waiting for CameraInfo fallback.')

    def _load_calibration_file(self, path: str) -> bool:
        if not os.path.exists(path):
            self.get_logger().warn(f'Calibration file not found: {path}')
            return False
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(f'Failed to read calibration file {path}: {exc}')
            return False

        camera_matrix = None
        distortion = None
        if isinstance(data.get('camera_matrix'), dict):
            camera_matrix = data['camera_matrix'].get('data')
        if isinstance(data.get('distortion_coefficients'), dict):
            distortion = data['distortion_coefficients'].get('data')
        camera_matrix = camera_matrix or data.get('k') or data.get('K')
        distortion = distortion if distortion is not None else data.get('d') or data.get('D') or []

        if camera_matrix is None or len(camera_matrix) != 9:
            self.get_logger().warn(
                f'Calibration file {path} is missing a 3x3 camera_matrix entry.')
            return False

        self.camera_matrix = np.array(camera_matrix, dtype=np.float64).reshape(3, 3)
        if distortion:
            self.dist_coeffs = np.array(distortion, dtype=np.float64).reshape(-1)
        else:
            self.dist_coeffs = np.zeros(5, dtype=np.float64)
        self.camera_source = path
        return True

    def _on_info(self, msg: CameraInfo):
        if self.camera_matrix is None:
            k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            d = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)
            self.camera_matrix = k
            self.dist_coeffs = d
            self.camera_source = 'camera_info'
            self.get_logger().info('Got CameraInfo, pose estimation enabled.')

    def _on_odom(self, msg: Odometry):
        self.last_odom = msg

    def _marker_confidence(self, corners: np.ndarray, frame_shape) -> float:
        height, width = frame_shape[:2]
        pts = corners.reshape(-1, 2).astype(np.float32)
        image_area = float(height * width)
        area_ratio = abs(cv2.contourArea(pts)) / image_area if image_area > 0.0 else 0.0
        area_score = clamp(area_ratio / self.area_reference, 0.0, 1.0)

        edge_lengths = []
        for i in range(4):
            edge_lengths.append(float(np.linalg.norm(pts[(i + 1) % 4] - pts[i])))
        max_edge = max(edge_lengths) if edge_lengths else 0.0
        square_score = min(edge_lengths) / max_edge if max_edge > 1e-6 else 0.0

        margin = min(
            float(np.min(pts[:, 0])),
            float(width - 1 - np.max(pts[:, 0])),
            float(np.min(pts[:, 1])),
            float(height - 1 - np.max(pts[:, 1])),
        )
        border_score = clamp(margin / 24.0, 0.0, 1.0)

        return round(0.5 * area_score + 0.3 * square_score + 0.2 * border_score, 3)

    def _publish_annotated(self, frame: np.ndarray, header):
        if self.annot_pub is None:
            return
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header = header
            self.annot_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'Failed to publish annotated frame: {exc}')

    def _on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return

        annotated = frame.copy() if self.annot_pub is not None else None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.dictionary, parameters=self.params)

        rvecs = tvecs = None
        if self.camera_matrix is not None and ids is not None and len(ids) > 0:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

        raw_detections = []
        confirmed_detections = []
        next_streaks: Dict[int, int] = {}

        if ids is not None and len(ids) > 0:
            for i, marker_id in enumerate(ids.flatten().tolist()):
                pts = corners[i].reshape(-1, 2).astype(np.int32)
                confidence = self._marker_confidence(corners[i], frame.shape)
                streak = 0
                if confidence >= self.min_confidence:
                    streak = self.detection_streaks.get(int(marker_id), 0) + 1
                    next_streaks[int(marker_id)] = streak

                entry = {
                    'id': int(marker_id),
                    'confidence': confidence,
                    'streak': streak,
                    'confirmed': streak >= self.required_consecutive,
                    'corners_px': pts.astype(float).tolist(),
                    'bbox_px': [
                        int(np.min(pts[:, 0])),
                        int(np.min(pts[:, 1])),
                        int(np.max(pts[:, 0]) - np.min(pts[:, 0])),
                        int(np.max(pts[:, 1]) - np.min(pts[:, 1])),
                    ],
                }
                if tvecs is not None:
                    t = tvecs[i].flatten().tolist()
                    r = rvecs[i].flatten().tolist()
                    entry['tvec'] = [float(v) for v in t]
                    entry['rvec'] = [float(v) for v in r]
                    entry['distance_m'] = float(np.linalg.norm(tvecs[i]))
                raw_detections.append(entry)
                if entry['confirmed']:
                    confirmed_detections.append(entry)

                if annotated is not None:
                    if entry['confirmed']:
                        color = (0, 220, 0)
                    elif confidence >= self.min_confidence:
                        color = (0, 200, 255)
                    else:
                        color = (0, 0, 255)
                    cv2.polylines(annotated, [pts], True, color, 2)
                    label = (
                        f'id={entry["id"]} conf={confidence:.2f} '
                        f'streak={streak}/{self.required_consecutive}'
                    )
                    anchor = tuple(pts[0])
                    cv2.putText(
                        annotated, label, anchor, cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, color, 1, cv2.LINE_AA)
                    if (self.camera_matrix is not None and
                            tvecs is not None and rvecs is not None):
                        cv2.drawFrameAxes(
                            annotated, self.camera_matrix, self.dist_coeffs,
                            rvecs[i], tvecs[i], self.marker_size * 0.35)

        self.detection_streaks = next_streaks

        robot_pose = None
        if self.last_odom is not None:
            p = self.last_odom.pose.pose
            robot_pose = {
                'x': float(p.position.x),
                'y': float(p.position.y),
                'yaw': float(quat_to_yaw(p.orientation.x, p.orientation.y,
                                         p.orientation.z, p.orientation.w)),
            }

        if self.annot_pub is not None:
            summary = (
                f'confirmed={len(confirmed_detections)} raw={len(raw_detections)} '
                f'calib={os.path.basename(self.camera_source)}'
            )
            cv2.putText(
                annotated, summary, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 255), 2, cv2.LINE_AA)
            self._publish_annotated(annotated, msg.header)

        if not confirmed_detections:
            return

        payload = {
            'stamp_sec': int(msg.header.stamp.sec),
            'stamp_nsec': int(msg.header.stamp.nanosec),
            'frame_id': msg.header.frame_id,
            'robot_pose': robot_pose,
            'camera_calibration_source': self.camera_source,
            'detections': confirmed_detections,
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
