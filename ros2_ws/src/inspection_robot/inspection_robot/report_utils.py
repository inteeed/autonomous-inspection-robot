"""Pure report helpers used by report_logger and tests."""
from __future__ import annotations

from typing import Dict, Iterable, List


def merge_detection_observation(
    observations: Dict[int, dict],
    marker_id: int,
    detection: dict,
    robot_pose,
    stamp: float,
    wall_time: float,
):
    distance = detection.get('distance_m')
    existing = observations.get(marker_id)
    if existing is None:
        observations[marker_id] = {
            'id': marker_id,
            'first_seen_stamp': stamp,
            'first_seen_wall': wall_time,
            'best_distance_m': distance,
            'best_tvec': detection.get('tvec'),
            'best_rvec': detection.get('rvec'),
            'best_robot_pose': robot_pose,
            'sightings': 1,
        }
        return observations[marker_id]

    existing['sightings'] += 1
    if (distance is not None and
            (existing['best_distance_m'] is None or distance < existing['best_distance_m'])):
        existing['best_distance_m'] = distance
        existing['best_tvec'] = detection.get('tvec')
        existing['best_rvec'] = detection.get('rvec')
        existing['best_robot_pose'] = robot_pose
    return existing


def _pdf_escape(text: str) -> str:
    return text.replace('\\', '\\\\').replace('(', '\\(').replace(')', '\\)')


def _wrap_lines(text: str, limit: int = 92) -> List[str]:
    words = str(text).split()
    lines = []
    current = ''
    for word in words:
        if len(current) + len(word) + 1 > limit:
            lines.append(current)
            current = word
        else:
            current = f'{current} {word}'.strip()
    if current:
        lines.append(current)
    return lines or ['']


def _report_lines(report: dict) -> Iterable[str]:
    yield 'Autonomous Inspection Report'
    yield f'Run ID: {report["run_id"]}'
    yield f'Generated: {report["generated_at"]}'
    yield f'Duration: {report["duration_s"]} s'
    yield f'Detected markers: {report["detected_markers"]}'
    yield f'Missing markers: {report["missing_markers"]}'
    yield ''
    anomaly_by_id = {int(a['id']): a for a in report.get('anomalies', [])}
    for marker_id in sorted(set(report['expected_markers']) | set(report['detected_markers'])):
        obs = next((o for o in report['observations'] if int(o['id']) == marker_id), None)
        anomaly = anomaly_by_id.get(marker_id, {})
        if obs is None:
            yield f'Marker {marker_id}: MISSING'
            continue
        status = 'FAIL' if anomaly.get('status') == 'ANOMALY' else 'PASS'
        yield f'Marker {marker_id}: {status}, sightings={obs["sightings"]}, distance={obs.get("best_distance_m")}'
        if anomaly:
            yield f'  anomaly_types={anomaly.get("types", [])} scores={anomaly.get("scores", {})}'
            if anomaly.get('snapshot_path'):
                yield f'  snapshot={anomaly["snapshot_path"]}'


def write_simple_pdf(path: str, report: dict):
    lines = []
    for line in _report_lines(report):
        lines.extend(_wrap_lines(line))

    page_lines = [lines[i:i + 42] for i in range(0, len(lines), 42)] or [[]]
    objects = []
    pages = []
    for chunk in page_lines:
        content = ['BT', '/F1 11 Tf', '50 770 Td', '14 TL']
        for line in chunk:
            content.append(f'({_pdf_escape(line)}) Tj')
            content.append('T*')
        content.append('ET')
        stream = '\n'.join(content)
        content_obj = len(objects) + 1
        objects.append(
            f'{content_obj} 0 obj\n'
            f'<< /Length {len(stream.encode("utf-8"))} >>\n'
            f'stream\n{stream}\nendstream\nendobj\n'
        )
        page_obj = len(objects) + 1
        pages.append(page_obj)
        objects.append(
            f'{page_obj} 0 obj\n'
            f'<< /Type /Page /Parent 0 0 R /MediaBox [0 0 612 792] '
            f'/Resources << /Font << /F1 0 0 R >> >> /Contents {content_obj} 0 R >>\n'
            f'endobj\n'
        )

    font_obj = len(objects) + 1
    objects.append(f'{font_obj} 0 obj\n<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>\nendobj\n')
    pages_obj = len(objects) + 1
    kids = ' '.join(f'{obj} 0 R' for obj in pages)
    objects.append(f'{pages_obj} 0 obj\n<< /Type /Pages /Kids [{kids}] /Count {len(pages)} >>\nendobj\n')
    catalog_obj = len(objects) + 1
    objects.append(f'{catalog_obj} 0 obj\n<< /Type /Catalog /Pages {pages_obj} 0 R >>\nendobj\n')

    fixed_objects = []
    for obj in objects:
        obj = obj.replace('/Parent 0 0 R', f'/Parent {pages_obj} 0 R')
        obj = obj.replace('/F1 0 0 R', f'/F1 {font_obj} 0 R')
        fixed_objects.append(obj)

    with open(path, 'wb') as f:
        f.write(b'%PDF-1.4\n')
        offsets = [0]
        for obj in fixed_objects:
            offsets.append(f.tell())
            f.write(obj.encode('utf-8'))
        xref = f.tell()
        f.write(f'xref\n0 {len(fixed_objects) + 1}\n'.encode('ascii'))
        f.write(b'0000000000 65535 f \n')
        for offset in offsets[1:]:
            f.write(f'{offset:010d} 00000 n \n'.encode('ascii'))
        f.write(
            f'trailer\n<< /Size {len(fixed_objects) + 1} /Root {catalog_obj} 0 R >>\n'
            f'startxref\n{xref}\n%%EOF\n'.encode('ascii')
        )
