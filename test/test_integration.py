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
from geometry_msgs.msg import TwistStamped
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
    process = launch_ros.actions.Node(
        package='wall_follower',
        executable='wall_follower_node',
        namespace='stamped',
        parameters=[{'start_enabled': False}],
        output='screen',
    )
    return (
        launch.LaunchDescription([
            process,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'controller_processes': [process]},
    )


class TestWallFollower(unittest.TestCase):
    """Check commands and states received after each test input."""

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
        for namespace in ('stamped',):
            endpoint = {'command': None, 'state': None, 'commands': [], 'states': []}
            self.endpoints[namespace] = endpoint

            def command_callback(message, endpoint=endpoint):
                self.assertEqual(message.header.frame_id, 'base_link')
                self.assertGreater(message.header.stamp.sec, 0)
                twist = message.twist
                endpoint['command'] = twist
                endpoint['commands'].append(twist)

            def state_callback(message, endpoint=endpoint):
                endpoint['state'] = message.data
                endpoint['states'].append(message.data)

            endpoint['publisher'] = self.node.create_publisher(
                LaserScan, f'/{namespace}/scan', qos_profile_sensor_data)
            endpoint['command_subscription'] = self.node.create_subscription(
                TwistStamped, f'/{namespace}/cmd_vel', command_callback, command_qos)
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
            description='discovery of the stamped controller and its service',
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
        """360 one-degree rays with configurable wall and front returns."""
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
            # Put several readings in each sector so the minimum and median are tested.
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
        """Send scans at 10 Hz and wait for the expected output to stay steady."""
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
        self.assert_stays_command(endpoint, state, self.stopped, duration=duration, scan=scan)

    def assert_stays_command(self, endpoint, state, predicate, *, duration=0.7, scan=None):
        """Wait for the expected output, then check every command for a set time."""
        self.expect(endpoint, state, predicate, scan=scan)
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
        self.assertTrue(all(predicate(c) for c in endpoint['commands']))
        self.assertEqual(set(endpoint['states']), {state})

    def test_controller_contract(self):
        def straight(command):
            return (abs(command.linear.x - 0.10) < 0.015
                    and abs(command.angular.z) < 0.08)

        for namespace, endpoint in self.endpoints.items():
            with self.subTest(namespace=namespace):
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
                def searching(command):
                    return (abs(command.linear.x - 0.10) < 0.01
                            and abs(command.angular.z + 0.325) < 0.02)

                all_nan = self.scan()
                all_nan.ranges = [math.nan] * 360
                self.expect(endpoint, 'searching_wall', searching, scan=all_nan)
                self.expect(endpoint, 'following_wall', straight,
                            scan=self.scan(front=math.nan))
                mixed_nan = self.scan()
                mixed_nan.ranges[90] = math.nan
                mixed_nan.ranges[135] = math.nan
                mixed_nan.ranges[180] = math.nan
                self.expect(endpoint, 'following_wall', straight, scan=mixed_nan)

                invalid = self.scan()
                invalid.ranges = [-math.inf] * 360
                malformed = self.scan()
                malformed.angle_increment = 0.0
                missing_sides = self.scan(front=0.15)
                missing_sides.angle_min = 0.0
                missing_sides.angle_max = 20 * missing_sides.angle_increment
                missing_sides.ranges = [0.15] * 21
                missing_right = self.scan(front=0.15)
                for index in range(80, 101):
                    missing_right.ranges[index] = -math.inf
                empty = self.scan()
                empty.ranges = []
                unusable = [invalid, malformed, missing_sides, missing_right, empty]

                # Reject a whole unusable scan, even if its front contains an obstacle.
                for bad_scan in unusable:
                    self.assert_stays_command(endpoint, 'invalid_scan', straight,
                                              duration=0.2, scan=bad_scan)
                self.expect(endpoint, 'following_wall', straight, scan=wall_scan)

                def turning(command):
                    return (abs(command.linear.x) < 1e-6
                            and abs(command.angular.z - 0.65) < 0.02)

                self.expect(endpoint, 'turning_left', turning, scan=self.scan(front=0.15))
                self.assert_stays_command(endpoint, 'invalid_scan', turning,
                                          duration=0.2, scan=invalid)
                self.expect(endpoint, 'turning_left', turning, scan=self.scan(front=0.30))
                self.expect(endpoint, 'following_wall', straight, scan=wall_scan)

                # Old or future timestamps should not change how a scan is handled.
                old_scan = self.scan()
                old_scan.header.stamp.sec = 1
                self.expect(endpoint, 'following_wall', straight, scan=old_scan)
                future_scan = self.scan(stamp=self.node.get_clock().now().to_msg())
                future_scan.header.stamp.sec += 3600
                self.expect(endpoint, 'following_wall', straight, scan=future_scan)
                # Commands and state should stay the same when scans stop arriving.
                self.assert_stays_command(endpoint, 'following_wall', straight, duration=1.3)
                self.expect(endpoint, 'turning_left', turning, scan=self.scan(front=0.15))

                self.enabled(endpoint, False)
                self.expect(endpoint, 'disabled', self.stopped, timeout=1.0)
                self.assert_stays_stopped(endpoint, 'disabled', scan=wall_scan)
                self.enabled(endpoint, True)
                self.assert_stays_stopped(endpoint, 'waiting_for_scan')
                # Neither old forward motion nor old turning may survive a restart.
                self.assert_stays_stopped(endpoint, 'invalid_scan', scan=invalid)
                self.expect(endpoint, 'following_wall', straight,
                            scan=self.scan(front=0.30))
                self.enabled(endpoint, True)
                self.assert_stays_command(endpoint, 'following_wall', straight, duration=0.2)
                self.enabled(endpoint, False)
                self.expect(endpoint, 'disabled', self.stopped)


@launch_testing.post_shutdown_test()
class TestControllerShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info, controller_processes):
        for process in controller_processes:
            launch_testing.asserts.assertExitCodes(proc_info, process=process)
