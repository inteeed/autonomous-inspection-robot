"""Lightweight visual anomaly detector around confirmed marker regions."""
from __future__ import annotations

import json
import os
import time
from datetime import datetime
from typing import Optional

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class AnomalyDetector(Node):
    def __init__(self):
        super().__init__('anomaly_detector')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/inspection/detections')
        self.declare_parameter('anomalies_topic', '/inspection/anomalies')
        self.declare_parameter('output_dir', os.path.expanduser('~/inspection_reports/anomalies'))
        self.declare_parameter('roi_padding_px', 45)
        self.declare_parameter('red_ratio_threshold', 0.035)
        self.declare_parameter('dark_ratio_threshold', 0.18)
        self.declare_parameter('save_snapshots', True)

        self.bridge = CvBridge()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_header = None
        self.output_dir = str(self.get_parameter('output_dir').value)
        os.makedirs(self.output_dir, exist_ok=True)

        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('detections_topic').value),
            self._on_detection,
            10,
        )
        self.pub = self.create_publisher(
            String, str(self.get_parameter('anomalies_topic').value), 10)
        self.get_logger().info('anomaly_detector ready.')

    def _on_image(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_header = msg.header
        except Exception as exc:
            self.get_logger().warn(f'Failed to decode image: {exc}')

    def _crop_roi(self, bbox):
        if self.latest_image is None or not bbox:
            return None
        x, y, w, h = [int(v) for v in bbox]
        pad = int(self.get_parameter('roi_padding_px').value)
        height, width = self.latest_image.shape[:2]
        x0 = max(0, x - pad)
        y0 = max(0, y - pad)
        x1 = min(width, x + w + pad)
        y1 = min(height, y + h + pad)
        if x1 <= x0 or y1 <= y0:
            return None
        return self.latest_image[y0:y1, x0:x1].copy()

    @staticmethod
    def _score_roi(roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        red_mask = (
            cv2.inRange(hsv, (0, 70, 60), (12, 255, 255)) |
            cv2.inRange(hsv, (165, 70, 60), (180, 255, 255))
        )
        dark_mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 55))
        area = float(roi.shape[0] * roi.shape[1])
        return {
            'red_ratio': float(np.count_nonzero(red_mask) / area) if area else 0.0,
            'dark_ratio': float(np.count_nonzero(dark_mask) / area) if area else 0.0,
        }

    def _save_snapshot(self, marker_id: int, roi) -> Optional[str]:
        if not bool(self.get_parameter('save_snapshots').value) or roi is None:
            return None
        name = f'marker_{marker_id}_{datetime.now().strftime("%Y%m%d_%H%M%S_%f")}.png'
        path = os.path.join(self.output_dir, name)
        cv2.imwrite(path, roi)
        return path

    def _on_detection(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        events = []
        for det in payload.get('detections', []):
            roi = self._crop_roi(det.get('bbox_px'))
            if roi is None:
                continue
            scores = self._score_roi(roi)
            red_bad = scores['red_ratio'] >= float(
                self.get_parameter('red_ratio_threshold').value)
            dark_bad = scores['dark_ratio'] >= float(
                self.get_parameter('dark_ratio_threshold').value)
            status = 'ANOMALY' if red_bad or dark_bad else 'NOMINAL'
            marker_id = int(det['id'])
            events.append({
                'id': marker_id,
                'status': status,
                'types': [
                    name for name, bad in (
                        ('red_warning_or_leak', red_bad),
                        ('dark_stain_or_corrosion', dark_bad),
                    ) if bad
                ],
                'scores': scores,
                'snapshot_path': self._save_snapshot(marker_id, roi),
            })
        if not events:
            return
        out = dict(payload)
        out['stamp_wall'] = time.time()
        out['anomalies'] = events
        msg_out = String()
        msg_out.data = json.dumps(out)
        self.pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
