"""Check the installed headless runtime without accepting a timed sleep."""

import os
import re
import time
import unittest

from controller_manager_msgs.srv import ListControllers
import launch
import launch.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from moveit_msgs.action import MoveGroup
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


def process_command_matches(pattern):
    """Return whether a live process command matches the byte pattern."""
    for entry in os.scandir('/proc'):
        if not entry.name.isdigit():
            continue

        try:
            with open(
                os.path.join(entry.path, 'cmdline'),
                'rb',
            ) as command_file:
                command = command_file.read().replace(b'\0', b' ')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue

        if re.search(pattern, command):
            return True

    return False


@pytest.mark.launch_test
def generate_test_description():
    sim_process = launch.actions.ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'sorting_arm_bringup',
            'sim.launch.xml',
            'gui:=false',
        ],
        output='screen',
    )

    return (
        launch.LaunchDescription(
            [
                sim_process,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {'sim_process': sim_process},
    )


class TestHeadlessRuntime(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('sorting_arm_headless_smoke')

    def tearDown(self):
        self.node.destroy_node()

    def test_runtime_becomes_ready(self, proc_info, sim_process):
        proc_info.assertWaitForStartup(sim_process, timeout=10)
        deadline = time.monotonic() + 150.0
        self._wait_for_clock_and_joint_states(deadline)
        self._wait_for_active_controllers(deadline)
        self._wait_for_move_action(deadline)
        for pattern in (
            rb'gz sim',
            rb'parameter_bridge',
            rb'robot_state_publisher',
            rb'move_group',
        ):
            self.assertTrue(
                process_command_matches(pattern),
                f'Required process exited before teardown: {pattern!r}',
            )
        self.assertFalse(
            process_command_matches(rb'(^|/)rviz2?( |$)'),
            'RViz must not run headlessly',
        )

    def _wait_for_clock_and_joint_states(self, deadline):
        clock_values = []
        joint_states = []
        clock_subscription = self.node.create_subscription(
            Clock,
            '/clock',
            lambda message: clock_values.append(
                message.clock.sec * 1_000_000_000 + message.clock.nanosec
            ),
            qos_profile_sensor_data,
        )
        joint_subscription = self.node.create_subscription(
            JointState,
            '/joint_states',
            joint_states.append,
            qos_profile_sensor_data,
        )

        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                clock_advanced = (
                    len(clock_values) >= 2 and clock_values[-1] > clock_values[0]
                )
                if clock_advanced and joint_states:
                    return
        finally:
            self.node.destroy_subscription(clock_subscription)
            self.node.destroy_subscription(joint_subscription)

        self.fail('Gazebo clock or joint states did not become ready')

    def _wait_for_active_controllers(self, deadline):
        client = self.node.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
        )
        required = {
            'joint_state_broadcaster',
            'arm_controller',
            'gripper_controller',
        }
        try:
            while time.monotonic() < deadline:
                if not client.wait_for_service(timeout_sec=0.5):
                    continue

                future = client.call_async(ListControllers.Request())
                remaining = max(0.0, deadline - time.monotonic())
                rclpy.spin_until_future_complete(
                    self.node,
                    future,
                    timeout_sec=min(remaining, 2.0),
                )
                if not future.done() or future.result() is None:
                    continue

                active = {
                    controller.name
                    for controller in future.result().controller
                    if controller.state == 'active'
                }
                if required <= active:
                    return
        finally:
            self.node.destroy_client(client)

        self.fail('The three required controllers did not become active')

    def _wait_for_move_action(self, deadline):
        client = ActionClient(self.node, MoveGroup, '/move_action')
        try:
            self.assertTrue(
                client.wait_for_server(
                    timeout_sec=max(0.0, deadline - time.monotonic())
                ),
                'MoveIt move_action server did not become ready',
            )
        finally:
            client.destroy()


@launch_testing.post_shutdown_test()
class TestHeadlessShutdown(unittest.TestCase):

    def test_launch_exits_cleanly(self, proc_info, sim_process):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=sim_process,
        )

    def test_launch_has_no_error_output(self, proc_output):
        output = ''.join(
            event.text.decode(errors='replace')
            for event in proc_output
        )
        self.assertNotRegex(output, re.compile(r'\b(?:ERROR|FATAL)\b'))

    def test_required_process_tree_stopped(self):
        required_processes = (
            rb'gz sim',
            rb'parameter_bridge',
            rb'robot_state_publisher',
            rb'move_group',
        )
        for pattern in required_processes:
            self.assertFalse(
                process_command_matches(pattern),
                f'Process remained after shutdown: {pattern!r}',
            )
