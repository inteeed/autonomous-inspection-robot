from inspection_robot.report_utils import merge_detection_observation, write_simple_pdf


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


def test_write_simple_pdf_creates_pdf(tmp_path):
    report = {
        'run_id': 'run_test',
        'generated_at': '2026-05-11T12:00:00',
        'duration_s': 12.3,
        'expected_markers': [0],
        'detected_markers': [0],
        'missing_markers': [],
        'observations': [{'id': 0, 'sightings': 1, 'best_distance_m': 1.4}],
        'anomalies': [{'id': 0, 'status': 'NOMINAL', 'types': [], 'scores': {}}],
    }
    path = tmp_path / 'report.pdf'
    write_simple_pdf(str(path), report)

    assert path.read_bytes().startswith(b'%PDF-1.4')
