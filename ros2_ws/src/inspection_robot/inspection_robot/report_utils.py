"""Pure report helpers used by report_logger and tests."""
from __future__ import annotations

import os
from typing import Dict, Iterable, List, Optional, Tuple


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


# ---------------------------------------------------------------------------
# PDF helpers
# ---------------------------------------------------------------------------

def _pdf_escape(text: str) -> str:
    return text.replace('\\', '\\\\').replace('(', '\\(').replace(')', '\\)')


def _wrap_lines(text: str, limit: int = 92) -> List[str]:
    words = str(text).split()
    lines: List[str] = []
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
    history = report.get('history', {})
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
        hist = history.get(str(marker_id), {})
        if obs is None:
            yield f'Marker {marker_id}: MISSING'
            continue
        severity = anomaly.get('severity', '')
        status_str = 'NOMINAL'
        if anomaly.get('status') == 'ANOMALY':
            status_str = f'ANOMALY [{severity}]'
        consec = hist.get('consecutive_anomalies', 0)
        trend = f'  (anomaly for {consec} consecutive runs)' if consec > 1 else ''
        yield (f'Marker {marker_id}: {status_str}'
               f'  sightings={obs["sightings"]}'
               f'  dist={obs.get("best_distance_m", "?"):.2f}m'
               + trend)
        if anomaly.get('types'):
            yield f'  findings: {", ".join(anomaly["types"])}'
            scores = anomaly.get('scores', {})
            if scores:
                score_str = '  scores: ' + '  '.join(
                    f'{k}={v:.3f}' for k, v in scores.items())
                yield score_str
            if anomaly.get('snapshot_path'):
                yield f'  snapshot: {anomaly["snapshot_path"]}'


def _jpeg_dimensions(path: str) -> Tuple[Optional[int], Optional[int]]:
    """Read image width and height from a JPEG file without external libs."""
    try:
        with open(path, 'rb') as f:
            data = f.read()
    except OSError:
        return None, None
    i = 0
    while i < len(data) - 1:
        if data[i] != 0xFF:
            break
        marker = data[i + 1]
        i += 2
        if marker in (0xC0, 0xC1, 0xC2):  # SOF0/SOF1/SOF2
            # length (2) + precision (1) + height (2) + width (2)
            if i + 7 > len(data):
                break
            h = (data[i + 3] << 8) | data[i + 4]
            w = (data[i + 5] << 8) | data[i + 6]
            return w, h
        if marker in (0xD8, 0xD9, 0x01) or (0xD0 <= marker <= 0xD7):
            continue  # fixed-size markers with no length field
        if i + 2 > len(data):
            break
        length = (data[i] << 8) | data[i + 1]
        i += length
    return None, None


def write_simple_pdf(path: str, report: dict):
    """Write a PDF report.  Anomaly snapshot JPEGs are embedded as image pages."""
    text_lines: List[str] = []
    for line in _report_lines(report):
        text_lines.extend(_wrap_lines(line))

    page_chunks = [text_lines[i:i + 42] for i in range(0, len(text_lines), 42)] or [[]]

    # Collect valid JPEG snapshots keyed by marker_id
    snapshot_entries: List[Tuple[int, str, bytes, int, int]] = []  # (id, path, data, w, h)
    for anomaly in report.get('anomalies', []):
        snap = anomaly.get('snapshot_path')
        if not snap or not os.path.isfile(snap):
            continue
        w, h = _jpeg_dimensions(snap)
        if w is None:
            continue
        try:
            with open(snap, 'rb') as f:
                jpeg_bytes = f.read()
        except OSError:
            continue
        snapshot_entries.append((int(anomaly['id']), snap, jpeg_bytes, w, h))

    # -----------------------------------------------------------------------
    # Build PDF object list
    # -----------------------------------------------------------------------
    objects: List[str] = []
    pages: List[int] = []
    img_xobj_ids: List[Tuple[int, int, int, int]] = []  # (marker_id, xobj_id, w, h)

    font_placeholder = 'FONT_OBJ_ID'
    pages_placeholder = 'PAGES_OBJ_ID'

    def next_id() -> int:
        return len(objects) + 1

    # --- Image XObjects (one per snapshot) ---
    for marker_id, snap_path, jpeg_bytes, w, h in snapshot_entries:
        xobj_id = next_id()
        img_xobj_ids.append((marker_id, xobj_id, w, h))
        objects.append(
            f'{xobj_id} 0 obj\n'
            f'<< /Type /XObject /Subtype /Image'
            f' /Width {w} /Height {h}'
            f' /ColorSpace /DeviceRGB /BitsPerComponent 8'
            f' /Filter /DCTDecode /Length {len(jpeg_bytes)} >>\n'
            f'stream\n'
        )
        # The JPEG bytes will be appended separately as raw bytes

    # --- Text pages ---
    for chunk in page_chunks:
        content_lines = ['BT', f'/{font_placeholder} 11 Tf', '50 770 Td', '14 TL']
        for line in chunk:
            content_lines.append(f'({_pdf_escape(line)}) Tj')
            content_lines.append('T*')
        content_lines.append('ET')
        stream = '\n'.join(content_lines)

        content_id = next_id()
        objects.append(
            f'{content_id} 0 obj\n'
            f'<< /Length {len(stream.encode("utf-8"))} >>\n'
            f'stream\n{stream}\nendstream\nendobj\n'
        )
        page_id = next_id()
        pages.append(page_id)
        objects.append(
            f'{page_id} 0 obj\n'
            f'<< /Type /Page /Parent {pages_placeholder} 0 R'
            f' /MediaBox [0 0 612 792]'
            f' /Resources << /Font << /F1 {font_placeholder} 0 R >> >>'
            f' /Contents {content_id} 0 R >>\n'
            f'endobj\n'
        )

    # --- Image pages (one per snapshot) ---
    for marker_id, xobj_id, w, h in img_xobj_ids:
        # Scale image to fit within 512×512 area centred on page
        max_side = 512
        scale = min(max_side / w, max_side / h)
        draw_w = int(w * scale)
        draw_h = int(h * scale)
        x_off = (612 - draw_w) // 2
        y_off = (792 - draw_h) // 2 - 40  # leave room for caption

        caption = f'Anomaly snapshot — Marker {marker_id}'
        stream = (
            f'BT /{font_placeholder} 10 Tf'
            f' {x_off} {y_off + draw_h + 12} Td'
            f' ({_pdf_escape(caption)}) Tj ET\n'
            f'q {draw_w} 0 0 {draw_h} {x_off} {y_off} cm'
            f' /Im{xobj_id} Do Q'
        )
        content_id = next_id()
        objects.append(
            f'{content_id} 0 obj\n'
            f'<< /Length {len(stream.encode("utf-8"))} >>\n'
            f'stream\n{stream}\nendstream\nendobj\n'
        )
        page_id = next_id()
        pages.append(page_id)
        objects.append(
            f'{page_id} 0 obj\n'
            f'<< /Type /Page /Parent {pages_placeholder} 0 R'
            f' /MediaBox [0 0 612 792]'
            f' /Resources'
            f' << /Font << /F1 {font_placeholder} 0 R >>'
            f'    /XObject << /Im{xobj_id} {xobj_id} 0 R >> >>'
            f' /Contents {content_id} 0 R >>\n'
            f'endobj\n'
        )

    # --- Font dictionary ---
    font_id = next_id()
    objects.append(
        f'{font_id} 0 obj\n'
        f'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>\n'
        f'endobj\n'
    )

    # --- Pages dictionary ---
    pages_id = next_id()
    kids = ' '.join(f'{p} 0 R' for p in pages)
    objects.append(
        f'{pages_id} 0 obj\n'
        f'<< /Type /Pages /Kids [{kids}] /Count {len(pages)} >>\n'
        f'endobj\n'
    )

    # --- Catalog ---
    catalog_id = next_id()
    objects.append(
        f'{catalog_id} 0 obj\n'
        f'<< /Type /Catalog /Pages {pages_id} 0 R >>\n'
        f'endobj\n'
    )

    # -----------------------------------------------------------------------
    # Resolve placeholders and write file
    # -----------------------------------------------------------------------
    def resolve(s: str) -> str:
        return (s
                .replace(f'/{font_placeholder} ', f'/F1 ')
                .replace(f'{font_placeholder} 0 R', f'{font_id} 0 R')
                .replace(f'{pages_placeholder} 0 R', f'{pages_id} 0 R'))

    # Map xobj_id → index in snapshot_entries for raw byte writing
    xobj_jpeg: Dict[int, bytes] = {xobj_id: jpeg_bytes
                                   for (_, snap_path, jpeg_bytes, _, _), (_, xobj_id, _, _)
                                   in zip(snapshot_entries, img_xobj_ids)}

    with open(path, 'wb') as f:
        f.write(b'%PDF-1.4\n')
        offsets: List[int] = [0]

        for idx, obj_str in enumerate(objects, start=1):
            offsets.append(f.tell())
            resolved = resolve(obj_str)

            # Image XObjects: write header as text then raw JPEG bytes then endstream/endobj
            matching_xobj = next(
                ((mid, xid, w, h) for mid, xid, w, h in img_xobj_ids if xid == idx),
                None
            )
            if matching_xobj is not None:
                _, xobj_id, _, _ = matching_xobj
                jpeg_bytes = xobj_jpeg[xobj_id]
                f.write(resolved.encode('utf-8'))
                f.write(jpeg_bytes)
                f.write(b'\nendstream\nendobj\n')
            else:
                f.write(resolved.encode('utf-8'))

        xref_offset = f.tell()
        total_objs = len(objects) + 1
        f.write(f'xref\n0 {total_objs}\n'.encode('ascii'))
        f.write(b'0000000000 65535 f \n')
        for offset in offsets[1:]:
            f.write(f'{offset:010d} 00000 n \n'.encode('ascii'))
        f.write((
            f'trailer\n<< /Size {total_objs} /Root {catalog_id} 0 R >>\n'
            f'startxref\n{xref_offset}\n%%EOF\n'
        ).encode('ascii'))
