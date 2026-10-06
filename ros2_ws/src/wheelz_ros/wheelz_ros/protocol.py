"""Existing ESP8266 wire protocol, independent of ROS for hardware-free testing."""
import http.client
import json
import math
import re
from urllib.parse import urlsplit

ACTIONS = {'forward', 'backward', 'left', 'right', 'stop'}
COUNTS = re.compile(r'Left:\s*(-?\d+)\s*\|\s*Right:\s*(-?\d+)\s*')


def action_for_twist(linear, angular, linear_deadband=0.01,
                     angular_deadband=0.05, positive_yaw_action='right'):
    """Angular commands take priority: the firmware cannot drive curved paths."""
    if not math.isfinite(linear) or not math.isfinite(angular):
        return 'stop'
    if abs(angular) > angular_deadband:
        opposite = 'left' if positive_yaw_action == 'right' else 'right'
        return positive_yaw_action if angular > 0 else opposite
    if abs(linear) > linear_deadband:
        return 'forward' if linear > 0 else 'backward'
    return 'stop'


def parse_counts(text):
    match = COUNTS.fullmatch(text.strip())
    if not match:
        raise ValueError('Invalid /counts response')
    counts = tuple(int(value) for value in match.groups())
    if any(not -(2**31) <= value < 2**31 for value in counts):
        raise ValueError('Encoder count outside ESP signed 32-bit range')
    return counts


def parse_imu(text, accel_scale=16384.0, gyro_scale=131.0):
    raw = json.loads(text)
    values = [raw[key] for key in ('ax', 'ay', 'az', 'gx', 'gy', 'gz')]
    if any(type(value) is not int or not -32768 <= value <= 32767
           for value in values):
        raise ValueError('Invalid MPU6050 raw sample')
    return (tuple(value * 9.80665 / accel_scale for value in values[:3]),
            tuple(math.radians(value / gyro_scale) for value in values[3:]))


class EspClient:
    def __init__(self, url, timeout=0.2):
        parts = urlsplit(url)
        if (parts.scheme != 'http' or not parts.hostname or parts.username
                or parts.password or parts.path not in ('', '/')
                or parts.query or parts.fragment):
            raise ValueError('robot_url must be http://HOST[:PORT] with no path or credentials')
        if not math.isfinite(timeout) or not 0.02 <= timeout <= 2.0:
            raise ValueError('HTTP timeout must be between 0.02 and 2 seconds')
        self.host, self.port, self.timeout = parts.hostname, parts.port or 80, timeout

    def get(self, path, no_reply_expected=False):
        connection = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout)
        try:
            # Do not follow redirects or use environment proxies for robot commands.
            connection.request('GET', path, headers={'Connection': 'close'})
            try:
                response = connection.getresponse()
            except (TimeoutError, http.client.RemoteDisconnected):
                if no_reply_expected:
                    # Existing /left and /right never call server.send().
                    # Transmission is NOT proof that the motor command executed.
                    return None
                raise
            if response.status != 200:
                raise OSError(f'{path}: HTTP {response.status}')
            payload = response.read(2049)
            if len(payload) > 2048:
                raise ValueError(f'{path}: oversized response')
            return payload.decode('utf-8')
        finally:
            connection.close()

    def command(self, action):
        if action not in ACTIONS:
            raise ValueError('Unknown action')
        return self.get('/' + action, no_reply_expected=action in ('left', 'right')) is not None

    def counts(self):
        return parse_counts(self.get('/counts'))

    def imu(self, accel_scale=16384.0, gyro_scale=131.0):
        return parse_imu(self.get('/imu'), accel_scale, gyro_scale)


def count_delta(current, previous):
    """Signed 32-bit wrap, matching the ESP's long counters."""
    return (current - previous + 2**31) % 2**32 - 2**31


class Odometry:
    def __init__(self, wheel_diameter, track_width, ticks_per_revolution,
                 left_sign=1, right_sign=1, max_speed=1.0):
        if any(not math.isfinite(v) or v <= 0 for v in
               (wheel_diameter, track_width, ticks_per_revolution, max_speed)):
            raise ValueError('Odometry dimensions, ticks and speed bound must be positive')
        if left_sign not in (-1, 1) or right_sign not in (-1, 1):
            raise ValueError('Encoder signs must be -1 or 1')
        self.meters_per_tick = math.pi * wheel_diameter / ticks_per_revolution
        self.track = track_width
        self.signs = (left_sign, right_sign)
        self.max_speed = max_speed
        self.x = self.y = self.yaw = 0.0
        self.previous = None

    def rebaseline(self):
        self.previous = None

    def update(self, counts, now):
        previous, self.previous = self.previous, (counts, now)
        if previous is None:
            return None
        old_counts, old_time = previous
        dt = now - old_time
        if dt <= 0 or dt > 2.0:
            return None
        left, right = [count_delta(new, old) * sign * self.meters_per_tick
                       for new, old, sign in zip(counts, old_counts, self.signs)]
        if max(abs(left), abs(right)) > self.max_speed * dt + 2 * self.meters_per_tick:
            # Counter reboot/corruption: establish a new baseline, never teleport.
            return None
        distance, angle = (left + right) / 2, (right - left) / self.track
        factor = math.sin(angle / 2) / (angle / 2) if abs(angle) > 1e-9 else 1.0
        self.x += distance * factor * math.cos(self.yaw + angle / 2)
        self.y += distance * factor * math.sin(self.yaw + angle / 2)
        self.yaw = (self.yaw + angle + math.pi) % (2 * math.pi) - math.pi
        return self.x, self.y, self.yaw, distance / dt, angle / dt
