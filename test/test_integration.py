"""Exercise the real controller with synthetic scans; no simulator is required."""

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool


@pytest.mark.launch_test
def generate_test_description():
    processes = [
        launch_ros.actions.Node(
            package='wall_follower',
            executable='wall_follower_node',
            namespace=namespace,
            parameters=[{
                'use_stamped_cmd_vel': stamped,
                'start_enabled': False,
                'scan_timeout': 0.5,
            }],
            output='screen',
        )
        for namespace, stamped in [('stamped', True), ('plain', False)]
    ]
    return (
        launch.LaunchDescription([
            *processes,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'controller_processes': processes},
    )


class TestWallFollower(unittest.TestCase):
    """Each expectation needs fresh state and command messages after its stimulus."""

    def setUp(self):
        self.context = Context()
        rclpy.init(context=self.context)
        self.addCleanup(self.context.try_shutdown)
        self.node = rclpy.create_node(
            'wall_follower_integration_client', context=self.context)
        self.addCleanup(self.node.destroy_node)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.addCleanup(self.executor.shutdown, timeout_sec=2.0)
        self.endpoints = {}
        command_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for namespace, message_type in [('stamped', TwistStamped), ('plain', Twist)]:
            endpoint = {'command': None, 'state': None, 'commands': [], 'states': []}
            self.endpoints[namespace] = endpoint

            def command_callback(message, endpoint=endpoint):
                twist = message.twist if isinstance(message, TwistStamped) else message
                endpoint['command'] = twist
                endpoint['commands'].append(twist)

            def state_callback(message, endpoint=endpoint):
                endpoint['state'] = message.data
                endpoint['states'].append(message.data)

            endpoint['publisher'] = self.node.create_publisher(
                LaserScan, f'/{namespace}/scan', qos_profile_sensor_data)
            endpoint['command_subscription'] = self.node.create_subscription(
                message_type, f'/{namespace}/cmd_vel', command_callback, command_qos)
            endpoint['state_subscription'] = self.node.create_subscription(
                String, f'/{namespace}/state', state_callback, state_qos)
            endpoint['service'] = self.node.create_client(
                SetBool, f'/{namespace}/set_enabled')
        self.wait_until(
            lambda: all(
                e['publisher'].get_subscription_count() > 0
                and e['command_subscription'].get_publisher_count() > 0
                and e['state_subscription'].get_publisher_count() > 0
                and e['service'].service_is_ready()
                for e in self.endpoints.values()
            ),
            timeout=15.0,
            description='discovery of both controllers and their services',
        )

    def wait_until(self, predicate, *, timeout=5.0, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        self.fail(f'Timed out waiting for {description}; '
                  f'states={[(k, e["state"]) for k, e in self.endpoints.items()]}')

    @staticmethod
    def scan(*, wall=0.25, front=math.inf, stamp=None):
        """360 one-degree rays; infinity means observed free space, NaN unknown."""
        message = LaserScan()
        message.header.frame_id = 'laser'
        if stamp is not None:
            message.header.stamp = stamp
        message.angle_min = -math.pi
        message.angle_increment = math.pi / 180.0
        message.angle_max = message.angle_min + 359 * message.angle_increment
        message.range_min = 0.05
        message.range_max = 10.0
        message.scan_time = 0.1
        ranges = [math.inf] * 360
        for index in range(360):
            degrees = index - 180
            # Give sector-based controllers several valid samples per direction.
            if wall is not None and -100 <= degrees <= -35:
                ranges[index] = wall / -math.sin(math.radians(degrees))
            if abs(degrees) <= 20:
                ranges[index] = front
        message.ranges = ranges
        return message

    @staticmethod
    def stopped(command):
        return all(abs(value) < 1e-6 for value in (
            command.linear.x, command.linear.y, command.linear.z,
            command.angular.x, command.angular.y, command.angular.z,
        ))

    def enabled(self, endpoint, value):
        future = endpoint['service'].call_async(SetBool.Request(data=value))
        try:
            self.wait_until(future.done, description=f'set_enabled({value}) response')
            response = future.result()
            self.assertIsNotNone(response)
            self.assertTrue(response.success, response.message)
        finally:
            if not future.done():
                endpoint['service'].remove_pending_request(future)
                future.cancel()

    def expect(self, endpoint, states, predicate, *, scan=None, timeout=5.0):
        """Stream at 10 Hz and require a stable result across multiple timer ticks."""
        states = {states} if isinstance(states, str) else set(states)
        endpoint['command'] = None
        endpoint['state'] = None
        next_scan = 0.0
        matched_since = None

        def matches():
            nonlocal next_scan, matched_since
            now = time.monotonic()
            if scan is not None and now >= next_scan:
                endpoint['publisher'].publish(scan)
                next_scan = now + 0.1
            if (endpoint['state'] in states and endpoint['command'] is not None
                    and predicate(endpoint['command'])):
                if matched_since is None:
                    matched_since = now
                return now - matched_since >= 0.15
            matched_since = None
            return False

        self.wait_until(matches, timeout=timeout, description=f'{states} and command')

    def assert_stays_stopped(self, endpoint, state, *, duration=0.7, scan=None):
        # Discard earlier transition messages, then inspect every received command.
        self.expect(endpoint, state, self.stopped, scan=scan)
        endpoint['commands'].clear()
        endpoint['states'].clear()
        deadline = time.monotonic() + duration
        next_scan = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if scan is not None and now >= next_scan:
                endpoint['publisher'].publish(scan)
                next_scan = now + 0.1
            self.executor.spin_once(timeout_sec=0.02)
        self.assertGreater(len(endpoint['commands']), 0)
        self.assertGreater(len(endpoint['states']), 0)
        self.assertTrue(all(self.stopped(c) for c in endpoint['commands']))
        self.assertEqual(set(endpoint['states']), {state})

    def test_controller_contract(self):
        def straight(command):
            return (abs(command.linear.x - 0.10) < 0.015
                    and abs(command.angular.z) < 0.08)

        for namespace, endpoint in self.endpoints.items():
            with self.subTest(output_format=namespace):
                wall_scan = self.scan()
                self.assertAlmostEqual(wall_scan.ranges[90], 0.25)
                self.assertAlmostEqual(wall_scan.ranges[135], 0.25 * math.sqrt(2))
                self.assert_stays_stopped(endpoint, 'disabled')
                self.assert_stays_stopped(endpoint, 'disabled', scan=wall_scan)
                self.enabled(endpoint, True)
                # Pre-enable scans must never start the freshly reset controller.
                self.assert_stays_stopped(endpoint, 'waiting_for_scan')
                self.expect(endpoint, 'following_wall', straight, scan=wall_scan)
                self.expect(
                    endpoint, 'following_wall',
                    lambda c: c.linear.x > 0.0 and c.angular.z > 0.02,
                    scan=self.scan(wall=0.15),
                )
                self.expect(
                    endpoint, 'following_wall',
                    lambda c: c.linear.x > 0.0 and c.angular.z < -0.02,
                    scan=self.scan(wall=0.40),
                )
                self.expect(
                    endpoint, 'turning_left',
                    lambda c: abs(c.linear.x) < 1e-6 and abs(c.angular.z - 0.65) < 0.02,
                    scan=self.scan(front=0.15),
                )
                self.expect(endpoint, 'following_wall', straight, scan=wall_scan)
                self.expect(
                    endpoint, 'searching_wall', lambda c: c.angular.z < -0.02,
                    scan=self.scan(wall=None),
                )
                invalid = self.scan()
                invalid.ranges = [math.nan] * 360
                self.expect(endpoint, 'invalid_scan', self.stopped, scan=invalid)
                self.expect(endpoint, 'following_wall', straight, scan=wall_scan)
                self.expect(
                    endpoint, {'partial_scan', 'invalid_scan'}, self.stopped,
                    scan=self.scan(front=math.nan),
                )
                self.expect(endpoint, 'following_wall', straight, scan=wall_scan)

                # Sensor timestamps must not control the steady receipt-time watchdog.
                old_scan = self.scan()
                old_scan.header.stamp.sec = 1
                self.expect(endpoint, 'following_wall', straight, scan=old_scan)
                future_scan = self.scan(stamp=self.node.get_clock().now().to_msg())
                future_scan.header.stamp.sec += 3600
                self.expect(endpoint, 'following_wall', straight, scan=future_scan)
                self.expect(endpoint, 'waiting_for_scan', self.stopped, timeout=2.0)
                self.expect(endpoint, 'following_wall', straight, scan=wall_scan)

                self.enabled(endpoint, False)
                self.expect(endpoint, 'disabled', self.stopped, timeout=1.0)
                self.assert_stays_stopped(endpoint, 'disabled', scan=wall_scan)
                self.enabled(endpoint, True)
                self.assert_stays_stopped(endpoint, 'waiting_for_scan')
                self.expect(endpoint, 'following_wall', straight, scan=wall_scan)
                self.enabled(endpoint, False)
                self.expect(endpoint, 'disabled', self.stopped)


@launch_testing.post_shutdown_test()
class TestControllerShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info, controller_processes):
        for process in controller_processes:
            launch_testing.asserts.assertExitCodes(proc_info, process=process)
