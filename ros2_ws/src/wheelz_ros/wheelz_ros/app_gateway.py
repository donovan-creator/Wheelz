"""Authenticated LAN API for Toolbox; the ROS policy and motor bridge stay on the PC."""
from concurrent.futures import Future
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import queue
import secrets
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from std_srvs.srv import SetBool

from .policy_contract import ACTION_NAMES
from .policy_node import PolicyNode


class ApiError(Exception):
    def __init__(self, message, status=409):
        super().__init__(message)
        self.status = status


class AppGateway(PolicyNode):
    def __init__(self):
        super().__init__()
        d = ParameterDescriptor(read_only=True)
        host = self.declare_parameter('api_host', '127.0.0.1', d).value
        port = self.declare_parameter('api_port', 8766, d).value
        self.origins = self.declare_parameter('allowed_origins', '', d).value.split(',')
        token_path = Path(self.declare_parameter(
            'token_file', str(Path.home() / '.config/wheelz/app-token'), d).value).expanduser()
        token_path.parent.mkdir(parents=True, exist_ok=True)
        if not token_path.exists():
            fd = os.open(token_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, 'w') as stream:
                stream.write(secrets.token_urlsafe(32))
        self.token = token_path.read_text().strip()
        if len(self.token) < 24:
            raise ValueError('App pairing token must contain at least 24 characters')
        self.jobs = queue.Queue()
        self.operation_lock = threading.Lock()
        self.generation = 0
        self.sequences = {}
        self.owner = ''
        self.last_heartbeat = 0.0
        self.manual_action = 0
        self.manual_time = 0.0
        self.driver_connected = False
        self.driver_enabled = False
        self.driver_seen = 0.0
        self.enable_driver = self.create_client(SetBool, '/wheelz_driver/enable')
        self.create_subscription(DiagnosticArray, 'diagnostics', self.on_diagnostics, 10)
        self.create_timer(.02, self.process_jobs)
        self.create_timer(.1, self.check_lease)
        self.http = ThreadingHTTPServer((host, port), handler_for(self))
        self.http.daemon_threads = True
        self.http_thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(f'App API listening on {host}:{self.http.server_port}; pairing token file: {token_path}')

    def on_diagnostics(self, message):
        for item in message.status:
            if item.name == 'wheelz/esp_bridge':
                fields = {kv.key: kv.value for kv in item.values}
                self.driver_connected = fields.get('connected') == 'True'
                self.driver_enabled = fields.get('enabled') == 'True'
                self.driver_seen = time.monotonic()

    def process_jobs(self):
        for _ in range(20):
            try:
                job, future, deadline = self.jobs.get_nowait()
            except queue.Empty:
                return
            if time.monotonic() > deadline:
                future.set_exception(ApiError('Request expired', 504))
                continue
            try:
                future.set_result(job())
            except Exception as error:
                future.set_exception(error)

    def invoke(self, job):
        future = Future()
        self.jobs.put((job, future, time.monotonic() + 2))
        return future.result(timeout=2.5)

    def driver_call(self, enabled):
        if not self.enable_driver.service_is_ready():
            raise ApiError('ROS motor bridge is unavailable', 503)
        request = SetBool.Request()
        request.data = enabled
        future = self.enable_driver.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(2):
            raise ApiError('Motor bridge did not acknowledge the request', 504)
        response = future.result()
        if not response.success:
            raise ApiError(response.message)
        return response.message

    def check_lease(self):
        now = time.monotonic()
        if self.owner and (self.active or self.manual_action):
            reason = None
            if now - self.last_heartbeat > 3:
                reason = 'app_disconnected'
            elif self.manual_action and now - self.manual_time > .6:
                reason = 'manual_released'
            elif not self.driver_connected or now - self.driver_seen > 1:
                reason = 'bridge_disconnected'
            elif not self.driver_enabled and now - self.manual_time > .7:
                reason = 'bridge_disabled'
            if reason:
                self.generation += 1
                self.manual_action = 0
                self.halt(reason)
                self.owner = ''
                request = SetBool.Request()
                request.data = False
                if self.enable_driver.service_is_ready():
                    self.enable_driver.call_async(request)
                return
        if self.manual_action:
            self.send(self.manual_action)

    def snapshot(self):
        online = self.driver_connected and time.monotonic() - self.driver_seen < 1
        fresh = self.fresh()
        return {
            'api_version': 1, 'connected': online,
            'ready': bool(online and fresh and self.enable_driver.service_is_ready()),
            'odometry_fresh': fresh, 'driver_enabled': self.driver_enabled,
            'status': self.status, 'active': bool(self.active and self.goal is not None),
            'owner': self.owner, 'frame': self.odom_frame, 'max_goal_distance_m': 4.0,
            'position': {'x': self.state[0], 'y': self.state[1], 'yaw': self.state[2]} if fresh else None,
            'goal': {'x': self.goal[0], 'y': self.goal[1]} if self.goal else None,
        }

    def stop_local(self, reason='cancelled'):
        self.manual_action = 0
        self.owner = ''
        self.halt(reason)

    def request(self, path, data):
        if path == '/status':
            return self.invoke(self.snapshot)
        if path == '/heartbeat':
            owner = data.get('client_id')
            def heartbeat():
                if self.owner and self.owner != owner:
                    raise ApiError('Another app controls this session')
                if self.owner:
                    self.last_heartbeat = time.monotonic()
                return self.snapshot()
            return self.invoke(heartbeat)
        if path not in ('/navigate', '/manual', '/stop'):
            raise ApiError('Unknown endpoint', 404)
        owner, sequence = data.get('client_id'), data.get('sequence')
        if not isinstance(owner, str) or not 1 <= len(owner) <= 128 or type(sequence) is not int or sequence < 1:
            raise ApiError('client_id and positive sequence are required', 400)
        def order_request():
            if sequence <= self.sequences.get(owner, 0):
                raise ApiError('Stale command rejected')
            self.sequences[owner] = sequence
            if len(self.sequences) > 256:
                oldest = next(iter(self.sequences))
                if oldest != self.owner:
                    self.sequences.pop(oldest)
        self.invoke(order_request)
        if path == '/stop':
            # Invalidate a pending navigation before waiting for the operation lock.
            self.invoke(lambda: setattr(self, 'generation', self.generation + 1))
        if not self.operation_lock.acquire(timeout=5):
            raise ApiError('Another command is still being stopped', 503)
        mutation_started = False
        try:
            if path == '/stop':
                self.invoke(self.stop_local)
                message = self.driver_call(False)
                return {'accepted': True, 'message': message, **self.invoke(self.snapshot)}
            owner = data.get('client_id')
            if not isinstance(owner, str) or not 1 <= len(owner) <= 128:
                raise ApiError('client_id is required', 400)
            def prepare():
                if self.owner and self.owner != owner and time.monotonic() - self.last_heartbeat < 3:
                    raise ApiError('Another app controls this session')
                if not self.driver_connected or time.monotonic() - self.driver_seen > 1:
                    raise ApiError('Robot bridge is offline', 503)
                if path == '/navigate':
                    x, y = data.get('x'), data.get('y')
                    if any(type(v) not in (int, float) or not math.isfinite(v) for v in (x, y)):
                        raise ApiError('X and Y must be finite numbers', 400)
                    if not self.fresh():
                        raise ApiError('Calibrated, fresh odometry is required')
                    if math.hypot(x - self.state[0], y - self.state[1]) > 4:
                        raise ApiError('Goal must be within 4 metres of the current position', 400)
                else:
                    if data.get('action') not in ACTION_NAMES:
                        raise ApiError('Invalid manual action', 400)
                return self.generation
            generation = self.invoke(prepare)
            mutation_started = True
            # Manual hold refreshes do not repeatedly reset the bridge's command queue.
            refresh = self.invoke(lambda: path == '/manual' and self.owner == owner
                                  and self.manual_action != 0 and self.driver_enabled)
            if not refresh:
                self.driver_call(True)
            def apply():
                if generation != self.generation or sequence != self.sequences.get(owner):
                    raise ApiError('Command cancelled')
                self.owner, self.last_heartbeat = owner, time.monotonic()
                self.manual_time = time.monotonic()
                if path == '/navigate':
                    if not self.fresh():
                        raise ApiError('Odometry became stale')
                    self.manual_action = 0
                    self.halt('waiting_for_goal')
                    self.active = True
                    goal = PoseStamped()
                    goal.header.frame_id = self.odom_frame
                    goal.pose.position.x, goal.pose.position.y = float(data['x']), float(data['y'])
                    self.on_goal(goal)
                    if self.goal is None:
                        raise ApiError('Goal was rejected')
                else:
                    self.halt('manual')
                    self.manual_action = ACTION_NAMES.index(data['action'])
                    self.send(self.manual_action)
                return {'accepted': True, **self.snapshot()}
            return self.invoke(apply)
        except Exception:
            if not mutation_started:
                raise
            # A failed/partially applied mutation must not leave an old goal running.
            self.invoke(lambda: self.stop_local('request_failed'))
            try:
                self.driver_call(False)
            except Exception:
                pass
            raise
        finally:
            self.operation_lock.release()

    def destroy_node(self):
        self.http.shutdown()
        self.http.server_close()
        self.http_thread.join(timeout=2)
        return super().destroy_node()


def handler_for(gateway):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, code, value):
            body = json.dumps(value, allow_nan=False).encode()
            self.send_response(code)
            origin = self.headers.get('Origin')
            if origin in gateway.origins:
                self.send_header('Access-Control-Allow-Origin', origin)
                self.send_header('Vary', 'Origin')
            self.send_header('Access-Control-Allow-Headers', 'Authorization, Content-Type')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_OPTIONS(self):
            self.reply(204, {})

        def dispatch(self, post):
            origin = self.headers.get('Origin')
            if origin and origin not in gateway.origins:
                self.reply(403, {'error': 'Browser origin is not allowed'})
                return
            expected = 'Bearer ' + gateway.token
            if not hmac.compare_digest(self.headers.get('Authorization', ''), expected):
                self.reply(401, {'error': 'Invalid pairing token'})
                return
            try:
                self.connection.settimeout(3)
                if post:
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 4096:
                        raise ApiError('Expected a small JSON request', 400)
                    data = json.loads(self.rfile.read(length))
                    if not isinstance(data, dict):
                        raise ApiError('Expected a JSON object', 400)
                    if self.path == '/status':
                        raise ApiError('Use GET for status', 405)
                else:
                    if self.path != '/status':
                        raise ApiError('Commands require POST', 405)
                    data = {}
                self.reply(200, gateway.request(self.path, data))
            except ApiError as error:
                self.reply(error.status, {'error': str(error)})
            except (ValueError, TypeError, json.JSONDecodeError):
                self.reply(400, {'error': 'Invalid request JSON'})
            except Exception as error:
                gateway.get_logger().warning(f'App request failed: {type(error).__name__}')
                self.reply(503, {'error': 'ROS request failed; check host status'})

        def do_GET(self):
            self.dispatch(False)

        def do_POST(self):
            self.dispatch(True)

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AppGateway()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
