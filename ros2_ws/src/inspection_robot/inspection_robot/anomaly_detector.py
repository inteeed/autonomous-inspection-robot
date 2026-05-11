"""Visual anomaly detector for confirmed marker regions.

Analyses the area around each detected ArUco marker for industrial defect
signals: warning indicators (red), corrosion/staining (dark), heat signatures
(orange), cracks (edge density), and electrical arcing/glare (brightness).

Severity is classified as NOMINAL / LOW / MEDIUM / HIGH based on how far each
ratio exceeds its threshold.  Any severity above NOMINAL is reported as ANOMALY.
"""
from __future__ import annotations

import json
import os
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Pure scoring helpers (importable / testable without ROS2)
# ---------------------------------------------------------------------------

def score_roi(roi: np.ndarray) -> Dict[str, float]:
    """Return a dict of anomaly signal ratios for a BGR image crop."""
    if roi is None or roi.size == 0:
        return {}
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    area = float(roi.shape[0] * roi.shape[1])

    # Red: two hue ranges to cover the wrap-around at 0/180
    red_mask = (
        cv2.inRange(hsv, (0, 70, 60), (12, 255, 255)) |
        cv2.inRange(hsv, (165, 70, 60), (180, 255, 255))
    )
    # Very dark (corrosion, oil, stain)
    dark_mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 50))
    # Orange/amber (heat signature, rust)
    orange_mask = cv2.inRange(hsv, (13, 100, 120), (28, 255, 255))
    # Very bright (sparks, electrical arcing, specular glare)
    bright_mask = cv2.inRange(hsv, (0, 0, 230), (180, 40, 255))

    # Crack proxy: Canny edge density (exclude pure black/white regions)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 60, 150)
    # Mask out the ArUco black/white pattern area (very high contrast) so we
    # measure only unexpected structural edges in the surrounding context.
    pattern_mask = cv2.inRange(gray, 20, 235)
    context_edges = cv2.bitwise_and(edges, edges, mask=pattern_mask)
    context_area = float(np.count_nonzero(pattern_mask)) or 1.0

    return {
        'red_ratio':    float(np.count_nonzero(red_mask)     / area),
        'dark_ratio':   float(np.count_nonzero(dark_mask)    / area),
        'orange_ratio': float(np.count_nonzero(orange_mask)  / area),
        'bright_ratio': float(np.count_nonzero(bright_mask)  / area),
        'edge_density': float(np.count_nonzero(context_edges) / context_area),
    }


def classify_severity(scores: Dict[str, float], thresholds: Dict[str, float]) -> Tuple[str, List[str]]:
    """Return (severity_label, [anomaly_type, ...]) from scores and thresholds."""
    findings: List[Tuple[str, float]] = []  # (type_name, excess_factor)
    labels = {
        'red_ratio':    'red_warning_or_leak',
        'dark_ratio':   'dark_stain_or_corrosion',
        'orange_ratio': 'heat_signature',
        'bright_ratio': 'electrical_arcing_or_glare',
        'edge_density': 'surface_cracking',
    }
    for key, label in labels.items():
        thr = thresholds.get(key)
        val = scores.get(key, 0.0)
        if thr and val >= thr:
            findings.append((label, val / thr))

    if not findings:
        return 'NOMINAL', []

    max_factor = max(f for _, f in findings)
    n = len(findings)
    if max_factor >= 5.0 or n >= 3:
        severity = 'HIGH'
    elif max_factor >= 3.0 or n >= 2:
        severity = 'MEDIUM'
    else:
        severity = 'LOW'

    return severity, [label for label, _ in findings]


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class AnomalyDetector(Node):
    def __init__(self):
        super().__init__('anomaly_detector')
        self.declare_parameter('image_topic',        '/camera/image_raw')
        self.declare_parameter('detections_topic',   '/inspection/detections')
        self.declare_parameter('anomalies_topic',    '/inspection/anomalies')
        self.declare_parameter('output_dir',
                               os.path.expanduser('~/inspection_reports/anomalies'))
        self.declare_parameter('roi_padding_px',            80)
        self.declare_parameter('red_ratio_threshold',       0.030)
        self.declare_parameter('dark_ratio_threshold',      0.150)
        self.declare_parameter('orange_ratio_threshold',    0.025)
        self.declare_parameter('bright_ratio_threshold',    0.040)
        self.declare_parameter('edge_density_threshold',    0.120)
        self.declare_parameter('save_snapshots',            True)

        self.bridge = CvBridge()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_header = None
        self.output_dir = str(self.get_parameter('output_dir').value)
        os.makedirs(self.output_dir, exist_ok=True)

        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            String, str(self.get_parameter('detections_topic').value),
            self._on_detection, 10)
        self.pub = self.create_publisher(
            String, str(self.get_parameter('anomalies_topic').value), 10)
        self.get_logger().info('anomaly_detector ready.')

    # ------------------------------------------------------------------
    def _on_image(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_header = msg.header
        except Exception as exc:
            self.get_logger().warn(f'Image decode failed: {exc}')

    def _crop_roi(self, bbox) -> Optional[np.ndarray]:
        if self.latest_image is None or not bbox:
            return None
        x, y, w, h = [int(v) for v in bbox]
        pad = int(self.get_parameter('roi_padding_px').value)
        ih, iw = self.latest_image.shape[:2]
        x0 = max(0, x - pad)
        y0 = max(0, y - pad)
        x1 = min(iw, x + w + pad)
        y1 = min(ih, y + h + pad)
        if x1 <= x0 or y1 <= y0:
            return None
        return self.latest_image[y0:y1, x0:x1].copy()

    def _thresholds(self) -> Dict[str, float]:
        return {
            'red_ratio':    float(self.get_parameter('red_ratio_threshold').value),
            'dark_ratio':   float(self.get_parameter('dark_ratio_threshold').value),
            'orange_ratio': float(self.get_parameter('orange_ratio_threshold').value),
            'bright_ratio': float(self.get_parameter('bright_ratio_threshold').value),
            'edge_density': float(self.get_parameter('edge_density_threshold').value),
        }

    def _save_snapshot(self, marker_id: int, roi: np.ndarray) -> Optional[str]:
        if not bool(self.get_parameter('save_snapshots').value) or roi is None:
            return None
        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        path = os.path.join(self.output_dir, f'marker_{marker_id}_{ts}.jpg')
        cv2.imwrite(path, roi, [cv2.IMWRITE_JPEG_QUALITY, 90])
        return path

    def _on_detection(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        thresholds = self._thresholds()
        events = []
        for det in payload.get('detections', []):
            roi = self._crop_roi(det.get('bbox_px'))
            if roi is None:
                continue
            scores = score_roi(roi)
            severity, types = classify_severity(scores, thresholds)
            status = 'ANOMALY' if severity != 'NOMINAL' else 'NOMINAL'
            marker_id = int(det['id'])

            if status == 'ANOMALY':
                self.get_logger().warn(
                    f'Marker {marker_id}: {severity} — {types} | '
                    + ' | '.join(f'{k}={v:.3f}(thr={thresholds[k]:.3f})'
                                 for k, v in scores.items() if k in thresholds and v > 0))

            events.append({
                'id':            marker_id,
                'status':        status,
                'severity':      severity,
                'types':         types,
                'scores':        scores,
                'snapshot_path': self._save_snapshot(marker_id, roi) if status == 'ANOMALY' else None,
            })

        if not events:
            return
        out = dict(payload)
        out['stamp_wall'] = time.time()
        out['anomalies'] = events
        pub_msg = String()
        pub_msg.data = json.dumps(out)
        self.pub.publish(pub_msg)


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
