"""Exercise the learned actor and the ROS policy's opt-in/stale-data behavior."""
import time
import unittest

import importlib.util

HAS_ROS = importlib.util.find_spec('rclpy') is not None
if HAS_ROS:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_srvs.srv import SetBool
    from wheelz_ros.policy_contract import TWIST_TARGETS, observation
    from wheelz_ros.policy_node import PolicyNode


@unittest.skipUnless(HAS_ROS, 'Requires a sourced ROS 2 installation')
class PolicyTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.policy = PolicyNode()
        self.client_node = Node('policy_test_client')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.policy)
        self.executor.add_node(self.client_node)
        self.odom_pub = self.client_node.create_publisher(Odometry, 'odom', 1)
        self.goal_pub = self.client_node.create_publisher(PoseStamped, 'goal_pose', 1)
        self.enable_client = self.client_node.create_client(SetBool, '/wheelz_policy/enable')

    def tearDown(self):
        self.executor.remove_node(self.policy)
        self.executor.remove_node(self.client_node)
        self.policy.destroy_node()
        self.client_node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.client_node.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)

    def wait(self, predicate, *, feed_odom=False, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if feed_odom:
                self.publish_odom()
            self.executor.spin_once(timeout_sec=.02)
            if predicate():
                return
        self.fail('Timed out waiting for policy state')

    def enable(self, value):
        self.wait(self.enable_client.service_is_ready)
        request = SetBool.Request()
        request.data = value
        future = self.enable_client.call_async(request)
        self.wait(future.done)
        return future.result()

    def goal(self, x, y, frame='odom'):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def test_actor_contract_and_nonfinite_rejection(self):
        obs = observation(0, 0, 0, 1, 0, 0, 0, 0, self.policy.cfg)
        self.assertIn(int(self.policy.policy.predict(obs)), range(5))
        for bad in ([float('nan')] * 9, [0] * 8):
            with self.assertRaises(ValueError):
                self.policy.policy.predict(bad)
        self.assertEqual(TWIST_TARGETS[3], (0.0, 0.5))
        self.assertEqual(TWIST_TARGETS[4], (0.0, -0.5))

    def test_disabled_invalid_goal_stale_odom_and_settled_goal(self):
        self.assertFalse(self.enable(True).success)
        self.wait(self.policy.fresh, feed_odom=True)
        self.assertTrue(self.enable(True).success)
        self.goal(1, 0, 'map')
        self.wait(lambda: self.policy.status == 'invalid_goal', feed_odom=True)
        self.assertFalse(self.policy.active)
        self.assertTrue(self.enable(True).success)
        self.goal(1, 0)
        self.wait(lambda: self.policy.goal is not None, feed_odom=True)
        self.wait(lambda: self.policy.status.startswith('navigating:'), feed_odom=True)
        self.wait(lambda: self.policy.status == 'stale_odometry', timeout=2)
        self.assertFalse(self.policy.active)
        self.assertIsNone(self.policy.goal)
        self.wait(self.policy.fresh, feed_odom=True)
        self.goal(1, 0)
        # Renewed odometry and a new goal must not silently re-enable.
        end = time.monotonic() + .15
        self.wait(lambda: time.monotonic() >= end, feed_odom=True)
        self.assertFalse(self.policy.active)
        self.assertTrue(self.enable(True).success)
        self.goal(0, 0)
        self.wait(lambda: self.policy.status == 'goal_reached', feed_odom=True)
        self.assertFalse(self.policy.active)
        self.assertEqual(self.policy.previous_action, 0)
        self.assertTrue(self.enable(False).success)


if __name__ == '__main__':
    unittest.main()
