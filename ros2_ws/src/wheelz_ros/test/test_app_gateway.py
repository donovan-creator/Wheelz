"""Real HTTP -> ROS -> simulated ESP tests; no physical hardware."""
import json
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import tempfile
import time
import unittest
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


@unittest.skipUnless(shutil.which('ros2'), 'Requires a sourced ROS installation')
class AppGatewayTests(unittest.TestCase):
    def test_api_auth_navigation_manual_cancel_and_lease(self):
        token = 'local-test-token-not-for-hardware-12345'
        with tempfile.TemporaryDirectory() as directory:
            token_file = Path(directory) / 'token'
            token_file.write_text(token)
            with socket.socket() as port_probe:
                port_probe.bind(('127.0.0.1', 0))
                port = port_probe.getsockname()[1]
            environment = dict(os.environ, ROS_DOMAIN_ID=str(50 + os.getpid() % 100),
                               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
            log = tempfile.TemporaryFile(mode='w+')
            process = subprocess.Popen([
                'ros2', 'launch', 'wheelz_ros', 'app.launch.py',
                f'api_port:={port}', f'token_file:={token_file}',
            ], env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            sequence = 0

            def request(path, body=None, auth=token):
                nonlocal sequence
                if body is not None and path != '/heartbeat':
                    sequence += 1
                    body = {'client_id': 'test-phone', 'sequence': sequence, **body}
                req = Request(f'http://127.0.0.1:{port}{path}',
                              data=None if body is None else json.dumps(body).encode(),
                              headers={'Authorization': 'Bearer ' + auth, 'Content-Type': 'application/json'})
                try:
                    with urlopen(req, timeout=9) as response:
                        return response.status, json.load(response)
                except HTTPError as error:
                    return error.code, json.load(error)

            def wait_for(predicate, timeout=10):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        self.fail('ROS launch exited')
                    try:
                        result = request('/status')[1]
                        if predicate(result):
                            return result
                    except (URLError, TimeoutError):
                        pass
                    time.sleep(.1)
                self.fail('Timeout waiting for app gateway state')

            try:
                wait_for(lambda state: state.get('ready'))
                self.assertEqual(request('/status', auth='wrong-token')[0], 401)
                self.assertEqual(request('/navigate', {'x': 'bad', 'y': 0})[0], 400)
                self.assertEqual(request('/navigate', {'x': 10, 'y': 0})[0], 400)
                code, result = request('/navigate', {'x': 1, 'y': 0})
                self.assertEqual(code, 200, result)
                self.assertTrue(result['active'])
                wait_for(lambda state: state['position']['x'] > .01, timeout=2)
                self.assertEqual(request('/heartbeat', {'client_id': 'test-phone'})[0], 200)
                # Unauthorized second session cannot take over or cancel the active goal.
                self.assertEqual(request('/manual', {'client_id': 'other-phone', 'action': 'forward'})[0], 409)
                self.assertTrue(request('/status')[1]['active'])
                code, result = request('/stop', {})
                self.assertEqual(code, 200, result)
                wait_for(lambda state: not state['driver_enabled'] and not state['active'])
                self.assertEqual(request('/manual', {'sequence': 1, 'action': 'forward'})[0], 409)
                code, result = request('/manual', {'action': 'backward'})
                self.assertEqual(code, 200, result)
                wait_for(lambda state: state['status'] == 'manual_released', timeout=2)
                wait_for(lambda state: not state['driver_enabled'])
                self.assertEqual(request('/navigate', {'x': 1, 'y': 0})[0], 200)
                wait_for(lambda state: state['status'] == 'app_disconnected', timeout=5)
                wait_for(lambda state: not state['driver_enabled'])
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read())
                raise
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
                log.close()
