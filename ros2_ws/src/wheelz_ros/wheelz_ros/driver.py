"""ROS topics and services over the unchanged Wheelz firmware."""
import math
import queue
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry as OdometryMsg
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Int64MultiArray, MultiArrayDimension, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

from .controller import Controller
from .protocol import EspClient, Odometry, action_for_twist


class Driver(Node):
    def __init__(self):
        super().__init__('wheelz_driver')
        defaults = {
            'robot_url': '', 'poll_hz': 10.0, 'http_timeout_s': 0.2,
            'command_timeout_s': 0.5, 'linear_deadband': 0.01,
            'angular_deadband': 0.05, 'positive_yaw_action': 'right',
            'wheel_diameter_m': 0.043, 'track_width_m': 0.16632,
            'ticks_per_revolution': 0.0, 'left_encoder_sign': 1,
            'right_encoder_sign': 1, 'max_plausible_wheel_speed_mps': 1.0,
            'accel_lsb_per_g': 16384.0, 'gyro_lsb_per_degree_s': 131.0,
            'base_frame': 'base_link', 'odom_frame': 'odom', 'imu_frame': 'imu_link',
            'publish_odom_tf': True,
        }
        self.cfg = {key: self.declare_parameter(
            key, value, ParameterDescriptor(read_only=True)).value
            for key, value in defaults.items()}
        c = self.cfg
        for key in ('poll_hz', 'command_timeout_s', 'accel_lsb_per_g', 'gyro_lsb_per_degree_s'):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        for key in ('linear_deadband', 'angular_deadband', 'ticks_per_revolution'):
            if not math.isfinite(c[key]) or c[key] < 0:
                raise ValueError(f'{key} must be finite and nonnegative')
        if c['poll_hz'] > 30 or c['command_timeout_s'] < 0.1:
            raise ValueError('poll_hz must be <=30; command_timeout_s must be >=0.1')
        if c['positive_yaw_action'] not in ('left', 'right'):
            raise ValueError('positive_yaw_action must be left or right')
        self.odometry = None
        if c['ticks_per_revolution'] > 0:
            self.odometry = Odometry(c['wheel_diameter_m'], c['track_width_m'],
                                     c['ticks_per_revolution'], c['left_encoder_sign'],
                                     c['right_encoder_sign'], c['max_plausible_wheel_speed_mps'])
        self.tf = TransformBroadcaster(self)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', qos_profile_sensor_data)
        self.counts_pub = self.create_publisher(Int64MultiArray, 'wheelz/encoder_counts', 10)
        self.odom_pub = self.create_publisher(OdometryMsg, 'odom', 10)
        self.action_pub = self.create_publisher(String, 'wheelz/action', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.controller = Controller(
            EspClient(c['robot_url'], c['http_timeout_s']), c['poll_hz'],
            c['command_timeout_s'], c['accel_lsb_per_g'], c['gyro_lsb_per_degree_s'])
        # Latest command only. A dead publisher must not leave a backlog of motion.
        self.create_subscription(Twist, 'cmd_vel', self.on_command, QoSProfile(depth=1))
        self.create_service(SetBool, '~/enable', self.on_enable)
        self.create_service(Trigger, '~/stop', self.on_stop)
        self.create_timer(0.02, self.publish_sample)
        self.create_timer(0.2, self.publish_status)
        self.epoch = None
        self.controller.start()
        self.get_logger().info('Started DISABLED. Enable with /wheelz_driver/enable.')
        self.get_logger().warning(
            'ESP has no disconnect watchdog. Wi-Fi or process loss can leave motors running.')
        if self.odometry is None:
            self.get_logger().info('Odometry disabled until ticks_per_revolution is calibrated.')

    def on_command(self, msg):
        components = (msg.linear.x, msg.linear.y, msg.linear.z,
                      msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(v) for v in components) or any(
                abs(v) > 1e-9 for v in (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)):
            self.controller.command('stop')
            return
        c = self.cfg
        self.controller.command(action_for_twist(
            msg.linear.x, msg.angular.z, c['linear_deadband'],
            c['angular_deadband'], c['positive_yaw_action']))

    def on_enable(self, request, response):
        response.success, response.message = self.controller.enable(request.data)
        return response

    def on_stop(self, request, response):
        response.success, response.message = self.controller.enable(False)
        return response

    def stamp_at(self, monotonic_sample):
        # Firmware has no timestamps; estimate host receive time in the ROS clock.
        age = max(0.0, time.monotonic() - monotonic_sample)
        return (self.get_clock().now() - Duration(seconds=age)).to_msg()

    def publish_sample(self):
        try:
            sample = self.controller.samples.get_nowait()
        except queue.Empty:
            return
        # Ignore a queued sample if a failure was detected after it was collected.
        if not self.controller.status()['connected']:
            return
        counts = Int64MultiArray()
        counts.layout.dim = [MultiArrayDimension(label='left,right', size=2, stride=2)]
        counts.data = list(sample.counts)
        self.counts_pub.publish(counts)
        imu = Imu()
        imu.header.stamp = self.stamp_at(sample.imu_time)
        imu.header.frame_id = self.cfg['imu_frame']
        imu.orientation.w = 1.0
        imu.orientation_covariance[0] = -1.0  # MPU raw data provides no orientation.
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = sample.acceleration
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = sample.gyro
        # Zero covariance means unknown, not a claim of noise-free measurements.
        self.imu_pub.publish(imu)
        if self.odometry is None:
            return
        if self.epoch != sample.epoch:
            self.odometry.rebaseline()
            self.epoch = sample.epoch
        result = self.odometry.update(sample.counts, sample.counts_time)
        if result is None:
            return
        x, y, yaw, linear, angular = result
        msg = OdometryMsg()
        msg.header.stamp = self.stamp_at(sample.counts_time)
        msg.header.frame_id = self.cfg['odom_frame']
        msg.child_frame_id = self.cfg['base_frame']
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = linear, angular
        # Conservative placeholders: measure uncertainty before sensor fusion.
        for index, value in zip((0, 7, 14, 21, 28, 35), (0.1, 0.1, 1e6, 1e6, 1e6, 0.2)):
            msg.pose.covariance[index] = value
            msg.twist.covariance[index] = value
        self.odom_pub.publish(msg)
        if self.cfg['publish_odom_tf']:
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = msg.child_frame_id
            transform.transform.translation.x, transform.transform.translation.y = x, y
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(transform)

    def publish_status(self):
        state = self.controller.status()
        self.action_pub.publish(String(data=state['last_action']))
        diagnostic = DiagnosticStatus()
        diagnostic.name = 'wheelz/esp_bridge'
        diagnostic.hardware_id = self.cfg['robot_url']
        diagnostic.level = DiagnosticStatus.WARN if state['connected'] else DiagnosticStatus.ERROR
        diagnostic.message = (
            'Connected; discrete control; no firmware watchdog' if state['connected']
            else state['error'])
        diagnostic.values = [KeyValue(key=key, value=str(value)) for key, value in state.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [diagnostic]
        self.diagnostics_pub.publish(msg)

    def destroy_node(self):
        self.controller.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Driver()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
