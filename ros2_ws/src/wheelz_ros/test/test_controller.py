import threading
import time
import unittest

from wheelz_ros.controller import Controller
from wheelz_ros.mock_esp import Robot, make_server
from wheelz_ros.protocol import EspClient


def wait_for(condition, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(0.01)
    raise AssertionError('Timed out waiting for condition')


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.robot = Robot()
        self.server = make_server(0, self.robot)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.client = EspClient(f'http://127.0.0.1:{self.server.server_port}', timeout=0.05)
        self.controller = Controller(self.client, poll_hz=30, command_timeout=0.15)

    def tearDown(self):
        if self.controller.thread.is_alive():
            self.controller.close()
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(timeout=2)

    def test_http_protocol_silent_turns_and_sensor_units(self):
        self.assertTrue(self.client.command('forward'))
        self.assertFalse(self.client.command('left'))
        self.assertEqual(self.robot.action, 'left')
        self.assertEqual(len(self.client.counts()), 2)
        self.assertAlmostEqual(self.client.imu()[0][2], 9.80665)
        self.assertTrue(self.client.command('stop'))

    def test_enable_timeout_fault_reconnect_and_shutdown(self):
        self.assertFalse(self.controller.enable(True)[0])
        self.controller.start()
        wait_for(lambda: self.controller.status()['connected'])
        self.controller.command('forward')
        time.sleep(0.08)
        self.assertEqual(self.robot.action, 'stop')
        self.assertTrue(self.controller.enable(True)[0])
        self.controller.command('forward')
        wait_for(lambda: self.robot.action == 'forward')
        wait_for(lambda: self.robot.action == 'stop')  # expired command
        self.controller.command('right')
        wait_for(lambda: self.robot.action == 'right')
        self.assertFalse(self.controller.status()['acknowledged'])
        self.robot.fail_sensors = True
        wait_for(lambda: not self.controller.status()['connected'])
        self.assertFalse(self.controller.status()['enabled'])
        wait_for(lambda: self.robot.action == 'stop')
        self.robot.fail_sensors = False
        wait_for(lambda: self.controller.status()['connected'])
        self.controller.command('forward')  # reconnection does not re-arm
        time.sleep(0.1)
        self.assertEqual(self.robot.action, 'stop')
        self.controller.enable(True)
        time.sleep(0.05)  # no replay of the command sent while disabled
        self.assertEqual(self.robot.action, 'stop')
        self.controller.command('backward')
        wait_for(lambda: self.robot.action == 'backward')
        self.controller.close()
        self.assertFalse(self.controller.thread.is_alive())
        self.assertEqual(self.robot.action, 'stop')


if __name__ == '__main__':
    unittest.main()
