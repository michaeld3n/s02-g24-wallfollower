"""Check velocity output from handmade scans, without a robot or simulator."""

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


@pytest.mark.launch_test
def generate_test_description():
    controller = launch_ros.actions.Node(
        package='wall_follower',
        executable='wall_follower_node',
        namespace='test_wall_follower',
        parameters=[{'start_enabled': True}],
        output='screen',
    )
    return launch.LaunchDescription([
        controller,
        launch_testing.actions.ReadyToTest(),
    ])


class TestWallFollower(unittest.TestCase):
    def setUp(self):
        self.context = Context()
        rclpy.init(context=self.context)
        self.addCleanup(self.context.try_shutdown)
        self.node = rclpy.create_node('wall_follower_test', context=self.context)
        self.addCleanup(self.node.destroy_node)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.addCleanup(self.executor.shutdown, timeout_sec=2.0)
        self.command = None
        self.publisher = self.node.create_publisher(
            LaserScan, '/test_wall_follower/scan', qos_profile_sensor_data)
        self.subscription = self.node.create_subscription(
            TwistStamped, '/test_wall_follower/cmd_vel', self.on_command, 10)

        # Wait for the test client and controller to find each other.
        deadline = time.monotonic() + 15.0
        while (self.publisher.get_subscription_count() == 0
               or self.subscription.get_publisher_count() == 0) and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
        self.assertGreater(self.publisher.get_subscription_count(), 0)
        self.assertGreater(self.subscription.get_publisher_count(), 0)

    def on_command(self, message):
        self.command = message.twist

    @staticmethod
    def scan(front, right):
        # One beam each for the right, front-right and front sectors.
        message = LaserScan()
        message.header.frame_id = 'laser'
        message.angle_min = -math.pi / 2
        message.angle_max = 0.0
        message.angle_increment = math.pi / 4
        message.range_min = 0.05
        message.range_max = 10.0
        message.ranges = [right, right * math.sqrt(2), front]
        return message

    def check_command(self, scan, linear, angular):
        self.command = None
        matched = False
        deadline = time.monotonic() + 5.0
        while not matched and time.monotonic() < deadline:
            self.publisher.publish(scan)
            self.executor.spin_once(timeout_sec=0.05)
            if self.command is not None:
                matched = (math.isclose(self.command.linear.x, linear, abs_tol=0.01)
                           and math.isclose(self.command.angular.z, angular, abs_tol=0.02))
        self.assertTrue(matched, f'Expected linear={linear}, angular={angular}; '
                                f'received {self.command}')

    def test_velocity_output(self):
        self.check_command(self.scan(front=2.0, right=0.25), linear=0.10, angular=0.0)
        self.check_command(self.scan(front=0.15, right=0.25), linear=0.0, angular=0.65)
        self.check_command(self.scan(front=math.inf, right=math.inf),
                           linear=0.10, angular=-0.325)
