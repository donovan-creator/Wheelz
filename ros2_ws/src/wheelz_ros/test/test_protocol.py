import json
import math
import unittest

from wheelz_ros.mjpeg import JpegBuffer
from wheelz_ros.protocol import (
    EspClient, Odometry, action_for_twist, count_delta, parse_counts, parse_imu,
)


class ProtocolTests(unittest.TestCase):
    def test_firmware_payloads_and_malformed_data(self):
        self.assertEqual(parse_counts('Left: -42 | Right: 99'), (-42, 99))
        for malformed in ('', 'Left: 1 | Right: 2 garbage', 'Left: 2147483648 | Right: 0'):
            with self.assertRaises(ValueError):
                parse_counts(malformed)
        raw = dict(ax=16384, ay=0, az=-16384, gx=131, gy=-131, gz=0)
        accel, gyro = parse_imu(json.dumps(raw))
        self.assertAlmostEqual(accel[0], 9.80665)
        self.assertAlmostEqual(accel[2], -9.80665)
        self.assertAlmostEqual(gyro[0], math.pi / 180)
        raw['gx'] = '131'
        with self.assertRaises(ValueError):
            parse_imu(json.dumps(raw))

    def test_discrete_commands_and_invalid_input(self):
        self.assertEqual(action_for_twist(0.2, 0), 'forward')
        self.assertEqual(action_for_twist(-0.2, 0), 'backward')
        self.assertEqual(action_for_twist(0.2, 0.3), 'right')
        self.assertEqual(action_for_twist(0, -0.3), 'left')
        self.assertEqual(action_for_twist(0, 0.3, positive_yaw_action='left'), 'left')
        for linear, angular in [(0.001, 0.001), (float('nan'), 1), (1, float('inf'))]:
            self.assertEqual(action_for_twist(linear, angular), 'stop')

    def test_url_validation(self):
        for url in ('', 'https://robot', 'http://robot/forward',
                    'http://user:password@robot', 'http://robot?command=go'):
            with self.assertRaises(ValueError):
                EspClient(url)
        self.assertEqual(EspClient('http://127.0.0.1:8080').port, 8080)

    def test_odometry_straight_rotation_reset_and_wrap(self):
        odom = Odometry(1 / math.pi, 0.5, 100, max_speed=2.0)
        self.assertIsNone(odom.update((0, 0), 0))
        result = odom.update((100, 100), 1)
        self.assertAlmostEqual(result[0], 1)
        self.assertAlmostEqual(result[3], 1)
        result = odom.update((75, 125), 2)
        self.assertAlmostEqual(result[0], 1)
        self.assertAlmostEqual(result[2], 1)
        self.assertAlmostEqual(result[4], 1)
        # Implausible jump rebaselines without moving the estimated pose.
        self.assertIsNone(odom.update((100000, 100000), 2.1))
        self.assertAlmostEqual(odom.x, 1)
        self.assertEqual(count_delta(-2147483648, 2147483647), 1)
        self.assertEqual(count_delta(2147483647, -2147483648), -1)
        odom.rebaseline()
        self.assertIsNone(odom.update((0, 0), 4))

    def test_encoder_polarity_and_missing_calibration(self):
        odom = Odometry(1 / math.pi, 0.5, 100, left_sign=-1)
        odom.update((0, 0), 0)
        self.assertAlmostEqual(odom.update((-50, 50), 1)[0], 0.5)
        with self.assertRaises(ValueError):
            Odometry(0.043, 0.166, 0)

    def test_mjpeg_split_markers_multiple_frames_and_bounds(self):
        parser = JpegBuffer(max_bytes=20)
        self.assertEqual(parser.feed(b'header\xff'), [])
        self.assertEqual(parser.feed(b'\xd8abc\xff'), [])
        self.assertEqual(parser.feed(b'\xd9\r\n\xff\xd8xyz\xff\xd9'),
                         [b'\xff\xd8abc\xff\xd9', b'\xff\xd8xyz\xff\xd9'])
        with self.assertRaises(ValueError):
            parser.feed(b'\xff\xd8' + b'a' * 30)
        self.assertEqual(parser.feed(b'junk\xff\xd8ok\xff\xd9'), [b'\xff\xd8ok\xff\xd9'])


if __name__ == '__main__':
    unittest.main()
