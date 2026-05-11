"""Tiny web dashboard for live inspection events."""
from __future__ import annotations

import json
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Inspection Dashboard</title>
  <style>
    body { margin: 0; font-family: sans-serif; background: #101418; color: #eef3f7; }
    main { max-width: 1100px; margin: 0 auto; padding: 24px; }
    h1 { margin: 0 0 18px; font-size: 28px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 14px; }
    .card { border: 1px solid #2d3945; border-radius: 8px; padding: 14px; background: #17202a; }
    .muted { color: #9caab7; font-size: 13px; }
    pre { white-space: pre-wrap; word-break: break-word; margin: 8px 0 0; font-size: 12px; }
    .ok { color: #7ce38b; }
    .bad { color: #ff8f7a; }
  </style>
</head>
<body>
<main>
  <h1>Inspection Dashboard</h1>
  <section class="grid">
    <div class="card"><div class="muted">Status</div><div id="status">waiting</div></div>
    <div class="card"><div class="muted">Detections</div><div id="detections">0</div></div>
    <div class="card"><div class="muted">Anomalies</div><div id="anomalies">0</div></div>
  </section>
  <section id="events" style="margin-top:14px;"></section>
</main>
<script>
const events = document.getElementById('events');
let detections = 0;
let anomalies = 0;
function addEvent(type, data) {
  const card = document.createElement('article');
  card.className = 'card';
  const title = document.createElement('div');
  title.innerHTML = `<strong>${type}</strong> <span class="muted">${new Date().toLocaleTimeString()}</span>`;
  const body = document.createElement('pre');
  body.textContent = JSON.stringify(data, null, 2);
  card.append(title, body);
  events.prepend(card);
  while (events.children.length > 80) events.removeChild(events.lastChild);
}
const source = new EventSource('/events');
source.onmessage = (event) => {
  const payload = JSON.parse(event.data);
  if (payload.type === 'status') document.getElementById('status').textContent = payload.data;
  if (payload.type === 'detection') {
    detections += (payload.data.detections || []).length;
    document.getElementById('detections').textContent = detections;
  }
  if (payload.type === 'anomaly') {
    anomalies += (payload.data.anomalies || []).filter(a => a.status === 'ANOMALY').length;
    document.getElementById('anomalies').textContent = anomalies;
  }
  addEvent(payload.type, payload.data);
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
        client = queue.Queue(maxsize=100)
        with self.lock:
            self.clients.append(client)
        return client

    def unregister(self, client):
        with self.lock:
            if client in self.clients:
                self.clients.remove(client)


class DashboardHandler(BaseHTTPRequestHandler):
    broker: EventBroker = None

    def log_message(self, fmt, *args):
        return

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(INDEX_HTML.encode('utf-8'))
            return
        if self.path == '/events':
            self._stream_events()
            return
        self.send_response(404)
        self.end_headers()

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
        self.broker = EventBroker()

        DashboardHandler.broker = self.broker
        self.httpd = ThreadingHTTPServer(
            (str(self.get_parameter('host').value), int(self.get_parameter('port').value)),
            DashboardHandler,
        )
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()

        self.create_subscription(String, '/inspection/status', self._on_status, 10)
        self.create_subscription(String, '/inspection/detections', self._on_detection, 10)
        self.create_subscription(String, '/inspection/anomalies', self._on_anomaly, 10)
        self.get_logger().info(
            f'dashboard_server listening on http://{self.get_parameter("host").value}:'
            f'{self.get_parameter("port").value}')

    def _publish_json_event(self, event_type: str, raw: str):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            data = raw
        self.broker.publish({'type': event_type, 'data': data})

    def _on_status(self, msg: String):
        self.broker.publish({'type': 'status', 'data': msg.data})

    def _on_detection(self, msg: String):
        self._publish_json_event('detection', msg.data)

    def _on_anomaly(self, msg: String):
        self._publish_json_event('anomaly', msg.data)

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
