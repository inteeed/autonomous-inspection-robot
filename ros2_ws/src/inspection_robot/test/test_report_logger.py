import numpy as np
import pytest

from inspection_robot.report_utils import merge_detection_observation, write_simple_pdf
from inspection_robot.anomaly_detector import score_roi, classify_severity


# ---------------------------------------------------------------------------
# merge_detection_observation
# ---------------------------------------------------------------------------

def test_merge_detection_observation_keeps_closest():
    observations = {}
    merge_detection_observation(
        observations,
        2,
        {'distance_m': 2.0, 'tvec': [2, 0, 0], 'rvec': [0, 0, 0]},
        {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
        10.0,
        100.0,
    )
    merge_detection_observation(
        observations,
        2,
        {'distance_m': 1.2, 'tvec': [1.2, 0, 0], 'rvec': [0, 1, 0]},
        {'x': 1.5, 'y': 0.0, 'yaw': 0.1},
        11.0,
        101.0,
    )

    obs = observations[2]
    assert obs['sightings'] == 2
    assert obs['first_seen_stamp'] == 10.0
    assert obs['best_distance_m'] == 1.2
    assert obs['best_tvec'] == [1.2, 0, 0]


def test_merge_detection_observation_ignores_farther_update():
    observations = {}
    merge_detection_observation(observations, 1, {'distance_m': 1.0}, None, 1.0, 1.0)
    merge_detection_observation(observations, 1, {'distance_m': 3.0}, None, 2.0, 2.0)

    assert observations[1]['sightings'] == 2
    assert observations[1]['best_distance_m'] == 1.0


def test_merge_detection_observation_handles_multiple_markers():
    observations = {}
    for detection in [{'id': 0, 'distance_m': 1.0}, {'id': 1, 'distance_m': 2.0}]:
        merge_detection_observation(
            observations,
            detection['id'],
            detection,
            {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            1.0,
            100.0,
        )

    assert observations[0]['best_distance_m'] == 1.0
    assert observations[1]['best_distance_m'] == 2.0


# ---------------------------------------------------------------------------
# write_simple_pdf
# ---------------------------------------------------------------------------

def _base_report(**kwargs):
    base = {
        'run_id': 'run_test',
        'generated_at': '2026-05-11T12:00:00',
        'duration_s': 12.3,
        'expected_markers': [0],
        'detected_markers': [0],
        'missing_markers': [],
        'observations': [{'id': 0, 'sightings': 1, 'best_distance_m': 1.4}],
        'anomalies': [{'id': 0, 'status': 'NOMINAL', 'types': [], 'scores': {}}],
    }
    base.update(kwargs)
    return base


def test_write_simple_pdf_creates_pdf(tmp_path):
    path = tmp_path / 'report.pdf'
    write_simple_pdf(str(path), _base_report())
    assert path.read_bytes().startswith(b'%PDF-1.4')


def test_write_simple_pdf_missing_snapshot_still_works(tmp_path):
    report = _base_report(anomalies=[{
        'id': 0, 'status': 'ANOMALY', 'severity': 'LOW',
        'types': ['red_warning_or_leak'], 'scores': {'red_ratio': 0.05},
        'snapshot_path': str(tmp_path / 'nonexistent.jpg'),
    }])
    path = tmp_path / 'report.pdf'
    write_simple_pdf(str(path), report)
    assert path.read_bytes().startswith(b'%PDF-1.4')


def test_write_simple_pdf_embeds_jpeg(tmp_path):
    # Create a tiny valid JPEG (10x10 red image) using numpy + cv2
    import cv2
    img = np.full((10, 10, 3), (0, 0, 200), dtype=np.uint8)
    snap = str(tmp_path / 'snap.jpg')
    cv2.imwrite(snap, img, [cv2.IMWRITE_JPEG_QUALITY, 70])

    report = _base_report(anomalies=[{
        'id': 0, 'status': 'ANOMALY', 'severity': 'LOW',
        'types': ['red_warning_or_leak'], 'scores': {'red_ratio': 0.05},
        'snapshot_path': snap,
    }])
    path = tmp_path / 'report.pdf'
    write_simple_pdf(str(path), report)
    data = path.read_bytes()
    assert data.startswith(b'%PDF-1.4')
    assert b'DCTDecode' in data   # JPEG stream is embedded


def test_write_simple_pdf_with_history(tmp_path):
    report = _base_report(history={
        '0': {'total_runs': 3, 'anomaly_runs': 2, 'last_status': 'ANOMALY',
              'consecutive_anomalies': 2, 'last_run': 'run_20260510_120000'},
    })
    path = tmp_path / 'report.pdf'
    write_simple_pdf(str(path), report)
    assert path.read_bytes().startswith(b'%PDF-1.4')


# ---------------------------------------------------------------------------
# score_roi / classify_severity
# ---------------------------------------------------------------------------

def _solid_bgr(b, g, r, size=40):
    return np.full((size, size, 3), (b, g, r), dtype=np.uint8)


def test_score_roi_red_patch():
    roi = _solid_bgr(0, 0, 220)
    scores = score_roi(roi)
    assert scores['red_ratio'] > 0.5, 'Pure red image should have high red_ratio'
    assert scores['dark_ratio'] < 0.1


def test_score_roi_dark_patch():
    roi = _solid_bgr(10, 8, 6)
    scores = score_roi(roi)
    assert scores['dark_ratio'] > 0.5, 'Very dark image should have high dark_ratio'


def test_score_roi_orange_patch():
    roi = _solid_bgr(0, 100, 240)
    scores = score_roi(roi)
    assert scores['orange_ratio'] > 0.3, 'Orange image should have high orange_ratio'


def test_score_roi_empty_returns_empty():
    assert score_roi(None) == {}
    assert score_roi(np.array([])) == {}


def test_classify_severity_nominal():
    thresholds = {'red_ratio': 0.03, 'dark_ratio': 0.15}
    severity, types = classify_severity({'red_ratio': 0.01, 'dark_ratio': 0.05}, thresholds)
    assert severity == 'NOMINAL'
    assert types == []


def test_classify_severity_low():
    thresholds = {'red_ratio': 0.03}
    severity, types = classify_severity({'red_ratio': 0.05}, thresholds)
    assert severity == 'LOW'
    assert 'red_warning_or_leak' in types


def test_classify_severity_medium_multi():
    thresholds = {'red_ratio': 0.03, 'dark_ratio': 0.15}
    severity, types = classify_severity({'red_ratio': 0.05, 'dark_ratio': 0.20}, thresholds)
    assert severity == 'MEDIUM'
    assert len(types) == 2


def test_classify_severity_high_factor():
    thresholds = {'red_ratio': 0.03}
    severity, types = classify_severity({'red_ratio': 0.18}, thresholds)  # 6× threshold
    assert severity == 'HIGH'
