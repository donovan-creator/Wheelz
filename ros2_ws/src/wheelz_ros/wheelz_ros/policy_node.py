"""Opt-in goal navigation. Publishes to a separate command topic by default."""
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .policy_contract import ACTION_NAMES, TWIST_TARGETS, observation
from .policy_runtime import DiscretePolicy


class PolicyNode(Node):
    def __init__(self):
        super().__init__('wheelz_policy')
        descriptor = ParameterDescriptor(read_only=True)
        default = str(Path(get_package_share_directory('wheelz_ros')) /
                      'models' / 'wheelz_discrete_policy.npz')
        path = self.declare_parameter('model_path', default, descriptor).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom', descriptor).value
        self.base_frame = self.declare_parameter('base_frame', 'base_link', descriptor).value
        self.max_age = self.declare_parameter('odom_timeout_s', 0.5, descriptor).value
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('odom_timeout_s must be positive')
        self.policy = DiscretePolicy(path)
        self.cfg = self.policy.config
        self.pub = self.create_publisher(Twist, 'wheelz/policy_cmd_vel', 1)
        self.status_pub = self.create_publisher(String, 'wheelz/policy_status', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 1)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_goal, 1)
        self.create_service(SetBool, '~/enable', self.on_enable)
        self.active = False
        self.goal = self.state = None
        self.received = 0.0
        self.previous_action = self.settled = 0
        self.started = 0.0
        self.status = 'disabled'
        self.create_timer(self.cfg['simulation']['control_dt_s'], self.tick)
        self.get_logger().info(
            'Policy disabled; output is /wheelz/policy_cmd_vel. Does not enable the motor bridge.')

    def fresh(self):
        if self.state is None:
            return False
        age = self.get_clock().now().nanoseconds / 1e9 - self.sample_stamp
        return time.monotonic() - self.received <= self.max_age and -0.1 <= age <= self.max_age

    def on_odom(self, msg):
        p, q, twist = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        values = (p.x, p.y, q.x, q.y, q.z, q.w, twist.linear.x, twist.angular.z)
        norm = sum(v * v for v in (q.x, q.y, q.z, q.w))
        if (msg.header.frame_id != self.odom_frame or msg.child_frame_id != self.base_frame
                or not all(math.isfinite(v) for v in values) or abs(norm - 1) > 0.1):
            self.state = None
            if self.active:
                self.halt('invalid_odometry')
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.state = (p.x, p.y, yaw, twist.linear.x, twist.angular.z)
        self.received = time.monotonic()
        self.sample_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    def on_goal(self, msg):
        if not self.active or not self.fresh():
            self.get_logger().warning('Enable the policy with fresh odometry before sending a goal.')
            return
        x, y = msg.pose.position.x, msg.pose.position.y
        max_distance = 2 * self.cfg['task']['world_half_extent_m']
        if (msg.header.frame_id != self.odom_frame or not math.isfinite(x) or not math.isfinite(y)
                or math.hypot(x - self.state[0], y - self.state[1]) > max_distance):
            self.halt('invalid_goal')
            return
        self.goal = (x, y)
        self.started = time.monotonic()
        self.previous_action = self.settled = 0
        self.status = 'navigating'

    def on_enable(self, request, response):
        if request.data and not self.fresh():
            response.success, response.message = False, 'Fresh calibrated odometry required'
            return response
        self.halt('waiting_for_goal' if request.data else 'disabled')
        self.active = request.data
        response.success = True
        response.message = 'Send a new goal_pose' if request.data else 'Policy disabled'
        return response

    def halt(self, reason):
        self.active = False
        self.goal = None
        self.previous_action = self.settled = 0
        self.status = reason
        self.send(0)

    def send(self, action):
        command = Twist()
        command.linear.x, command.angular.z = TWIST_TARGETS[action]
        self.pub.publish(command)
        self.status_pub.publish(String(data=self.status))

    def tick(self):
        if not self.active:
            # Do not fight another controller while this policy is disabled.
            self.status_pub.publish(String(data=self.status))
            return
        if not self.fresh():
            self.halt('stale_odometry')
            return
        if self.goal is None:
            self.send(0)
            return
        if time.monotonic() - self.started > self.cfg['task']['max_episode_steps'] * self.cfg['simulation']['control_dt_s']:
            self.halt('goal_timeout')
            return
        x, y, yaw, linear, angular = self.state
        distance = math.hypot(self.goal[0] - x, self.goal[1] - y)
        if distance <= self.cfg['task']['goal_tolerance_m']:
            # Goal stop guard is independent of actor predictions.
            wheel_speed_bound = abs(linear) + abs(angular) * self.cfg['robot']['track_width_m'] / 2
            self.settled = self.settled + 1 if wheel_speed_bound < self.cfg['task']['settled_speed_mps'] else 0
            self.previous_action = 0
            self.send(0)
            if self.settled >= self.cfg['task']['settle_steps']:
                self.halt('goal_reached')
            return
        self.settled = 0
        try:
            obs = observation(x, y, yaw, *self.goal, linear, angular,
                              self.previous_action, self.cfg)
            action = int(self.policy.predict(obs))
        except (ValueError, FloatingPointError) as error:
            self.get_logger().error(str(error))
            self.halt('inference_error')
            return
        self.previous_action = action
        self.status = 'navigating:' + ACTION_NAMES[action]
        self.send(action)

    def destroy_node(self):
        if rclpy.ok():
            self.halt('shutdown')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PolicyNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
