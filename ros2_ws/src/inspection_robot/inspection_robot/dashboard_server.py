"""Live inspection dashboard — HTTP + Server-Sent Events.

Endpoints:
  GET /               — dashboard HTML
  GET /events         — SSE stream (status, detection, anomaly events)
  GET /snapshot/<name>  — serve a saved anomaly snapshot JPEG
"""
from __future__ import annotations

import json
import mimetypes
import os
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

_SNAPSHOT_DIR = os.path.expanduser('~/inspection_reports/anomalies')

INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Inspection Dashboard</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:system-ui,sans-serif;background:#0d1117;color:#e6edf3;min-height:100vh}
    header{background:#161b22;border-bottom:1px solid #30363d;padding:16px 24px;
           display:flex;align-items:center;gap:12px}
    header h1{font-size:18px;font-weight:600}
    .dot{width:10px;height:10px;border-radius:50%;background:#238636;
         animation:pulse 2s infinite}
    @keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}
    main{max-width:1200px;margin:0 auto;padding:24px;display:grid;gap:16px}
    .kpi-row{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px}
    .kpi{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:16px}
    .kpi .label{font-size:11px;color:#8b949e;text-transform:uppercase;letter-spacing:.5px}
    .kpi .value{font-size:32px;font-weight:700;margin-top:4px}
    .ok{color:#3fb950}.bad{color:#f85149}.warn{color:#d29922}.muted{color:#8b949e}
    #status-text{font-size:13px;color:#58a6ff}
    .panels{display:grid;grid-template-columns:1fr 1fr;gap:16px}
    @media(max-width:700px){.panels{grid-template-columns:1fr}}
    .panel{background:#161b22;border:1px solid #30363d;border-radius:8px;
           padding:16px;overflow:hidden}
    .panel h2{font-size:13px;font-weight:600;color:#8b949e;margin-bottom:12px;
              text-transform:uppercase;letter-spacing:.5px}
    #marker-table{width:100%;border-collapse:collapse;font-size:13px}
    #marker-table th{text-align:left;padding:6px 8px;border-bottom:1px solid #30363d;
                     color:#8b949e;font-weight:500}
    #marker-table td{padding:6px 8px;border-bottom:1px solid #21262d}
    .badge{display:inline-block;padding:2px 8px;border-radius:12px;font-size:11px;
           font-weight:600}
    .badge-ok{background:#0d3320;color:#3fb950;border:1px solid #238636}
    .badge-bad{background:#3d0d0d;color:#f85149;border:1px solid #8b1a1a}
    .badge-warn{background:#2d2000;color:#d29922;border:1px solid #9e6a03}
    .badge-grey{background:#21262d;color:#8b949e;border:1px solid #30363d}
    #events-list{max-height:340px;overflow-y:auto;display:flex;flex-direction:column;gap:8px}
    .evt{background:#0d1117;border:1px solid #21262d;border-radius:6px;padding:10px 12px;
         font-size:12px}
    .evt .hdr{display:flex;justify-content:space-between;margin-bottom:4px}
    .evt .type{font-weight:600;font-size:11px;text-transform:uppercase;letter-spacing:.4px}
    .evt pre{white-space:pre-wrap;word-break:break-word;color:#8b949e;font-size:11px;
             max-height:80px;overflow:hidden}
    #snapshots{display:flex;flex-wrap:wrap;gap:10px;margin-top:8px}
    .snap-card{background:#0d1117;border:1px solid #30363d;border-radius:6px;
               overflow:hidden;width:180px}
    .snap-card img{width:100%;display:block}
    .snap-card .snap-label{font-size:11px;color:#8b949e;padding:4px 8px}
  </style>
</head>
<body>
<header>
  <div class="dot"></div>
  <h1>Industrial Inspection Dashboard</h1>
  <span id="status-text" style="margin-left:auto">connecting…</span>
</header>
<main>
  <div class="kpi-row">
    <div class="kpi">
      <div class="label">Status</div>
      <div class="value" id="kpi-status" style="font-size:16px;margin-top:8px">—</div>
    </div>
    <div class="kpi">
      <div class="label">Detections</div>
      <div class="value ok" id="kpi-det">0</div>
    </div>
    <div class="kpi">
      <div class="label">Anomalies</div>
      <div class="value bad" id="kpi-anom">0</div>
    </div>
    <div class="kpi">
      <div class="label">Events</div>
      <div class="value muted" id="kpi-evt">0</div>
    </div>
  </div>

  <div class="panels">
    <div class="panel">
      <h2>Marker Status</h2>
      <table id="marker-table">
        <thead><tr><th>ID</th><th>Sightings</th><th>Distance</th><th>Status</th></tr></thead>
        <tbody id="marker-tbody"></tbody>
      </table>
    </div>
    <div class="panel">
      <h2>Event Log</h2>
      <div id="events-list"></div>
    </div>
  </div>

  <div class="panel" id="snap-panel" style="display:none">
    <h2>Anomaly Snapshots</h2>
    <div id="snapshots"></div>
  </div>
</main>
<script>
const markers = {};
let detCount = 0, anomCount = 0, evtCount = 0;

function badge(status, severity) {
  const s = (status || '').toUpperCase();
  const sev = (severity || '').toUpperCase();
  if (s === 'ANOMALY') {
    const cls = sev === 'HIGH' ? 'badge-bad' : sev === 'MEDIUM' ? 'badge-warn' : 'badge-warn';
    return `<span class="badge ${cls}">${sev || 'ANOMALY'}</span>`;
  }
  if (s === 'NOMINAL' || s === 'PASS') return `<span class="badge badge-ok">PASS</span>`;
  if (s === 'MISSING') return `<span class="badge badge-grey">MISSING</span>`;
  return `<span class="badge badge-grey">${s}</span>`;
}

function refreshMarkerTable() {
  const tbody = document.getElementById('marker-tbody');
  const ids = Object.keys(markers).map(Number).sort((a,b)=>a-b);
  tbody.innerHTML = ids.map(id => {
    const m = markers[id];
    return `<tr>
      <td>${id}</td>
      <td>${m.sightings}</td>
      <td>${m.dist !== null ? m.dist.toFixed(2)+'m' : '—'}</td>
      <td>${badge(m.status, m.severity)}</td>
    </tr>`;
  }).join('');
}

function addEvent(type, data) {
  evtCount++;
  document.getElementById('kpi-evt').textContent = evtCount;
  const list = document.getElementById('events-list');
  const div = document.createElement('div');
  div.className = 'evt';
  const typeColor = type === 'anomaly' ? '#f85149' : type === 'detection' ? '#3fb950' : '#8b949e';
  div.innerHTML = `<div class="hdr">
    <span class="type" style="color:${typeColor}">${type}</span>
    <span class="muted">${new Date().toLocaleTimeString()}</span>
  </div><pre>${JSON.stringify(data, null, 2).substring(0, 300)}</pre>`;
  list.prepend(div);
  while (list.children.length > 60) list.removeChild(list.lastChild);
}

function addSnapshot(markerId, snapshotName, severity) {
  const panel = document.getElementById('snap-panel');
  panel.style.display = '';
  const container = document.getElementById('snapshots');
  const card = document.createElement('div');
  card.className = 'snap-card';
  card.innerHTML = `<img src="/snapshot/${encodeURIComponent(snapshotName)}" alt="marker ${markerId}">
    <div class="snap-label">Marker ${markerId} · ${severity || 'ANOMALY'}</div>`;
  container.prepend(card);
}

const source = new EventSource('/events');
source.onopen = () => {
  document.getElementById('status-text').textContent = 'connected';
};
source.onerror = () => {
  document.getElementById('status-text').textContent = 'reconnecting…';
};
source.onmessage = (event) => {
  const payload = JSON.parse(event.data);
  const type = payload.type;
  const data = payload.data;

  if (type === 'status') {
    document.getElementById('kpi-status').textContent = data;
    return;
  }
  if (type === 'heartbeat') return;

  addEvent(type, data);

  if (type === 'detection') {
    for (const det of (data.detections || [])) {
      detCount++;
      const id = det.id;
      if (!markers[id]) markers[id] = {sightings: 0, dist: null, status: 'NOMINAL', severity: ''};
      markers[id].sightings++;
      if (det.distance_m != null) markers[id].dist = det.distance_m;
    }
    document.getElementById('kpi-det').textContent = detCount;
    refreshMarkerTable();
  }

  if (type === 'anomaly') {
    for (const a of (data.anomalies || [])) {
      const id = a.id;
      if (!markers[id]) markers[id] = {sightings: 0, dist: null, status: 'NOMINAL', severity: ''};
      if (a.status === 'ANOMALY') {
        anomCount++;
        markers[id].status = 'ANOMALY';
        markers[id].severity = a.severity || '';
        if (a.snapshot_path) {
          const name = a.snapshot_path.split('/').pop();
          addSnapshot(id, name, a.severity);
        }
      } else {
        if (markers[id].status !== 'ANOMALY') markers[id].status = 'NOMINAL';
      }
    }
    document.getElementById('kpi-anom').textContent = anomCount;
    refreshMarkerTable();
  }
};
</script>
</body>
</html>
"""


class EventBroker:
    def __init__(self):
        self.clients = []
        self.lock = threading.Lock()

    def publish(self, event):
        with self.lock:
            clients = list(self.clients)
        for client in clients:
            try:
                client.put_nowait(event)
            except queue.Full:
                pass

    def register(self):
        client = queue.Queue(maxsize=200)
        with self.lock:
            self.clients.append(client)
        return client

    def unregister(self, client):
        with self.lock:
            if client in self.clients:
                self.clients.remove(client)


class DashboardHandler(BaseHTTPRequestHandler):
    broker: EventBroker = None
    snapshot_dir: str = _SNAPSHOT_DIR

    def log_message(self, fmt, *args):
        return

    def do_GET(self):
        if self.path == '/':
            self._serve_html()
        elif self.path == '/events':
            self._stream_events()
        elif self.path.startswith('/snapshot/'):
            self._serve_snapshot()
        else:
            self.send_response(404)
            self.end_headers()

    def _serve_html(self):
        body = INDEX_HTML.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_snapshot(self):
        filename = self.path[len('/snapshot/'):]
        # Prevent path traversal
        filename = os.path.basename(filename)
        filepath = os.path.join(self.snapshot_dir, filename)
        if not os.path.isfile(filepath):
            self.send_response(404)
            self.end_headers()
            return
        mime, _ = mimetypes.guess_type(filepath)
        mime = mime or 'application/octet-stream'
        try:
            with open(filepath, 'rb') as f:
                data = f.read()
        except OSError:
            self.send_response(500)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'max-age=3600')
        self.end_headers()
        self.wfile.write(data)

    def _stream_events(self):
        client = self.broker.register()
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        try:
            while True:
                try:
                    event = client.get(timeout=15.0)
                except queue.Empty:
                    event = {'type': 'heartbeat', 'data': {'stamp': time.time()}}
                line = f'data: {json.dumps(event)}\n\n'
                self.wfile.write(line.encode('utf-8'))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            self.broker.unregister(client)


class DashboardServer(Node):
    def __init__(self):
        super().__init__('dashboard_server')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('snapshot_dir',
                               os.path.expanduser('~/inspection_reports/anomalies'))

        self.broker = EventBroker()
        DashboardHandler.broker = self.broker
        DashboardHandler.snapshot_dir = str(self.get_parameter('snapshot_dir').value)

        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        self.httpd = ThreadingHTTPServer((host, port), DashboardHandler)
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()

        self.create_subscription(String, '/inspection/status',     self._on_status,    10)
        self.create_subscription(String, '/inspection/detections', self._on_detection, 10)
        self.create_subscription(String, '/inspection/anomalies',  self._on_anomaly,   10)
        self.get_logger().info(f'dashboard_server at http://{host}:{port}')

    def _broadcast(self, event_type: str, raw: str):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            data = raw
        self.broker.publish({'type': event_type, 'data': data})

    def _on_status(self, msg: String):
        self.broker.publish({'type': 'status', 'data': msg.data})

    def _on_detection(self, msg: String):
        self._broadcast('detection', msg.data)

    def _on_anomaly(self, msg: String):
        self._broadcast('anomaly', msg.data)

    def destroy_node(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
