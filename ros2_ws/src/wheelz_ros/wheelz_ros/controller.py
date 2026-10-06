"""Serialized network worker with no ROS dependencies.

Only this worker talks to the single-client ESP server. ROS callbacks never wait
for network I/O. Host timeouts are best-effort stops, not firmware watchdogs.
"""
from dataclasses import dataclass
import queue
import threading
import time

from .protocol import ACTIONS


@dataclass(frozen=True)
class Sample:
    counts: tuple
    counts_time: float
    acceleration: tuple
    gyro: tuple
    imu_time: float
    epoch: int


class Controller:
    def __init__(self, client, poll_hz=10.0, command_timeout=0.5,
                 accel_scale=16384.0, gyro_scale=131.0):
        self.client = client
        self.period = 1.0 / poll_hz
        self.command_timeout = command_timeout
        self.accel_scale, self.gyro_scale = accel_scale, gyro_scale
        self.lock = threading.Lock()
        self.quit = threading.Event()
        self.samples = queue.Queue(maxsize=1)
        self.enabled = False
        self.connected = False
        self.action = 'stop'
        self.command_time = 0.0
        self.last_sent = None
        self.last_sent_time = 0.0
        self.acknowledged = False
        self.error = 'Waiting for ESP'
        self.epoch = 0
        self.thread = threading.Thread(target=self._run, name='wheelz-http', daemon=True)

    def start(self):
        self.thread.start()

    def enable(self, enabled):
        with self.lock:
            if enabled and not self.connected:
                return False, 'Cannot enable: waiting for successful stop and sensor reads'
            self.enabled = enabled
            # Commands received while disabled or before an enable call never execute.
            self.action, self.command_time = 'stop', 0.0
        return True, 'Enabled; send a fresh cmd_vel' if enabled else 'Disabled; stop requested'

    def command(self, action):
        if action not in ACTIONS:
            action = 'stop'
        with self.lock:
            if self.enabled:
                self.action, self.command_time = action, time.monotonic()

    def status(self):
        with self.lock:
            return dict(connected=self.connected, enabled=self.enabled,
                        last_action=self.last_sent or 'unknown',
                        acknowledged=self.acknowledged, error=self.error)

    def _send_current(self, force=False):
        now = time.monotonic()
        with self.lock:
            action = self.action
            if not self.enabled or now - self.command_time >= self.command_timeout:
                action = 'stop'
            if not force and action == self.last_sent and now - self.last_sent_time < 0.5:
                return
        acknowledged = self.client.command(action)
        with self.lock:
            self.last_sent, self.last_sent_time = action, time.monotonic()
            self.acknowledged = acknowledged

    def _fault(self, error):
        with self.lock:
            self.connected = self.enabled = False
            self.action, self.command_time = 'stop', 0.0
            self.last_sent = None
            self.error = str(error)
            self.epoch += 1
        try:
            self._send_current(force=True)
        except Exception:
            pass  # Unreachable hardware cannot be stopped from the host.

    def _run(self):
        while not self.quit.is_set():
            start = time.monotonic()
            try:
                self._send_current()
                counts = self.client.counts()
                counts_time = time.monotonic()
                # Recheck the timeout between sensor requests, not only once per loop.
                self._send_current()
                acceleration, gyro = self.client.imu(self.accel_scale, self.gyro_scale)
                imu_time = time.monotonic()
                self._send_current()
                with self.lock:
                    self.connected, self.error = True, ''
                    epoch = self.epoch
                sample = Sample(counts, counts_time, acceleration, gyro, imu_time, epoch)
                try:
                    self.samples.get_nowait()
                except queue.Empty:
                    pass
                self.samples.put_nowait(sample)
            except Exception as error:
                self._fault(error)
            self.quit.wait(max(0.01, self.period - (time.monotonic() - start)))
        try:
            self.client.command('stop')
        except Exception:
            pass

    def close(self):
        self.enable(False)
        self.quit.set()
        self.thread.join(timeout=8.0)
