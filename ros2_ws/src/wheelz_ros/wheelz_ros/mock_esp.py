"""Loopback-only emulator of the existing ESP HTTP API, including silent turns."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time

from .protocol import ACTIONS


class Robot:
    def __init__(self):
        self.lock = threading.Lock()
        self.action = 'stop'
        self.left = self.right = 0.0
        self.last = time.monotonic()
        self.history = []
        self.fail_sensors = False

    def update(self, action=None):
        with self.lock:
            now = time.monotonic()
            # Artificial calibration: 360 counts / 43mm wheel revolution.
            speed = 0.15 * 360 / (math.pi * 0.043)
            left, right = {'stop': (0, 0), 'forward': (1, 1),
                           'backward': (-1, -1), 'left': (1, -1),
                           'right': (-1, 1)}[self.action]
            self.left += left * speed * (now - self.last)
            self.right += right * speed * (now - self.last)
            self.last = now
            if action is not None:
                self.action = action
                self.history.append((now, action))
            return round(self.left), round(self.right)


def make_server(port=8080, robot=None):
    robot = robot or Robot()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            action = self.path.lstrip('/')
            if action in ACTIONS:
                robot.update(action)
                if action in ('left', 'right'):
                    # Real turn handlers send no HTTP response.
                    self.close_connection = True
                    return
                self.reply(action)
            elif self.path == '/counts':
                if robot.fail_sensors:
                    self.send_error(503)
                    return
                left, right = robot.update()
                self.reply(f'Left: {left} | Right: {right}')
            elif self.path == '/imu':
                if robot.fail_sensors:
                    self.send_error(503)
                    return
                self.reply(json.dumps(dict(ax=0, ay=0, az=16384, gx=0, gy=0, gz=0)),
                           'application/json')
            elif self.path == '/ping':
                self.reply('pong')
            elif self.path == '/stream':
                self.stream_camera()
            else:
                self.send_error(404)

        def stream_camera(self):
            # Optional on Windows core tests; provided by ROS's python3-opencv.
            import cv2
            import numpy as np
            frame = np.zeros((120, 160, 3), dtype=np.uint8)
            frame[:, :, 1] = 100
            ok, jpeg = cv2.imencode('.jpg', frame)
            if not ok:
                self.send_error(500)
                return
            jpeg = jpeg.tobytes()
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                    self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def reply(self, value, content_type='text/plain'):
            body = value.encode()
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Connection', 'close')
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()
    server = make_server(args.port)
    print(f'Mock ESP on http://127.0.0.1:{server.server_port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
