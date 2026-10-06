"""Real DDS/launch integration against loopback ESPs; never physical hardware."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'Requires a sourced ROS 2 installation')
class RosIntegrationTests(unittest.TestCase):
    def test_launch_topics_camera_services_and_command_timeout(self):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, Imu, CompressedImage
        from std_msgs.msg import Int64MultiArray, String
        from std_srvs.srv import SetBool, Trigger

        old_domain = os.environ.get('ROS_DOMAIN_ID')
        os.environ['ROS_DOMAIN_ID'] = str(50 + os.getpid() % 100)
        # Keep test ROS traffic inside WSL.
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        rclpy.init()
        node = rclpy.create_node('wheelz_integration_test')
        messages = {}
        subscriptions = []
        for topic, kind, qos in [
            ('odom', Odometry, 10), ('imu/data_raw', Imu, qos_profile_sensor_data),
            ('wheelz/encoder_counts', Int64MultiArray, 10), ('wheelz/action', String, 10),
            ('camera/image_raw', Image, qos_profile_sensor_data),
            ('camera/image_raw/compressed', CompressedImage, qos_profile_sensor_data),
        ]:
            subscriptions.append(node.create_subscription(
                kind, topic, lambda msg, topic=topic: messages.__setitem__(topic, msg), qos))
        publisher = node.create_publisher(Twist, 'cmd_vel', 1)
        enable = node.create_client(SetBool, '/wheelz_driver/enable')
        stop = node.create_client(Trigger, '/wheelz_driver/stop')
        log = tempfile.TemporaryFile(mode='w+')
        process = subprocess.Popen(
            ['ros2', 'launch', 'wheelz_ros', 'bringup.launch.py', 'mock:=true'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        def spin_until(predicate, timeout=15.0, publish=None):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise AssertionError('Bringup exited early')
                if publish is not None:
                    publisher.publish(publish)
                rclpy.spin_once(node, timeout_sec=0.05)
                if predicate():
                    return
            raise AssertionError('Timed out waiting for ROS result')

        def call(client, request):
            future = client.call_async(request)
            spin_until(future.done)
            self.assertTrue(future.result().success, future.result().message)

        try:
            spin_until(lambda: len(messages) == 6 and enable.service_is_ready()
                       and stop.service_is_ready())
            self.assertEqual(messages['wheelz/action'].data, 'stop')
            self.assertEqual(messages['imu/data_raw'].orientation_covariance[0], -1)
            self.assertAlmostEqual(messages['imu/data_raw'].linear_acceleration.z, 9.80665)
            self.assertEqual(messages['camera/image_raw'].width, 160)
            self.assertEqual(messages['camera/image_raw'].height, 120)
            self.assertGreater(len(messages['camera/image_raw/compressed'].data), 100)
            request = SetBool.Request()
            request.data = True
            call(enable, request)
            command = Twist()
            command.linear.x = 0.1
            spin_until(lambda: messages['odom'].pose.pose.position.x > 0.04,
                       publish=command)
            self.assertEqual(messages['wheelz/action'].data, 'forward')
            # Stop publishing: bridge must issue /stop without another Twist.
            spin_until(lambda: messages['wheelz/action'].data == 'stop', timeout=3.0)
            command.linear.x = 0.0
            command.angular.z = 0.3
            spin_until(lambda: messages['wheelz/action'].data == 'right', publish=command)
            spin_until(lambda: messages['odom'].pose.pose.orientation.z > 0.03, publish=command)
            call(stop, Trigger.Request())
            command.linear.x, command.angular.z = 0.1, 0.0
            spin_until(lambda: messages['wheelz/action'].data == 'stop', publish=command)
            # Disabled state persists despite fresh motion commands.
            end = time.monotonic() + 0.3
            spin_until(lambda: time.monotonic() > end, publish=command)
            self.assertEqual(messages['wheelz/action'].data, 'stop')
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
            node.destroy_node()
            rclpy.shutdown()
            if old_domain is None:
                os.environ.pop('ROS_DOMAIN_ID', None)
            else:
                os.environ['ROS_DOMAIN_ID'] = old_domain


if __name__ == '__main__':
    unittest.main()
