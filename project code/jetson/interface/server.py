import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit


class StatusServer:
    def __init__(self, config, store):
        html = (Path(__file__).parent / "dashboard.html").read_bytes()
        class Handler(BaseHTTPRequestHandler):
            def setup(self):
                super().setup()
                self.connection.settimeout(2)

            def do_GET(self):
                path = urlsplit(self.path).path
                if path == "/":
                    payload, content_type = html, "text/html; charset=utf-8"
                elif path == "/api/status":
                    snapshot = store.read()
                    snapshot["served_monotonic_s"] = time.monotonic()
                    payload = json.dumps(snapshot, ensure_ascii=False, allow_nan=False).encode("utf-8")
                    content_type = "application/json; charset=utf-8"
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.end_headers()
                try:
                    self.wfile.write(payload)
                except (OSError, TimeoutError):
                    pass

            def log_message(self, *_):
                pass

        self.server = ThreadingHTTPServer((config.host, config.port), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": .1},
                                       name="status-http", daemon=True)

    @property
    def url(self):
        host, port = self.server.server_address
        return f"http://{'127.0.0.1' if host == '0.0.0.0' else host}:{port}/"

    def start(self):
        self.thread.start()

    def close(self):
        if self.thread.is_alive():
            self.server.shutdown()
            self.thread.join(timeout=2)
        self.server.server_close()
