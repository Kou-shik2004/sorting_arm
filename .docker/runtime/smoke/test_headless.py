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
from moveit_msgs.action import MoveGroup
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from sorting_arm_interfaces.action import Home
from sorting_arm_interfaces.action import Pick
from sorting_arm_interfaces.action import Place
from sorting_arm_interfaces.srv import DetectObjects
from sorting_arm_interfaces.srv import SyncObjects


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
    app_process = launch.actions.ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'sorting_arm_bringup',
            'app.launch.xml',
            'gui:=false',
            'show_viewer:=false',
        ],
        output='screen',
    )

    return (
        launch.LaunchDescription(
            [
                app_process,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {'app_process': app_process},
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

    def test_runtime_becomes_ready(self, proc_info, proc_output, app_process):
        proc_info.assertWaitForStartup(app_process, timeout=10)
        deadline = time.monotonic() + 180.0
        self._wait_for_clock_and_joint_states(deadline)
        self._wait_for_active_controllers(deadline)
        self._wait_for_move_action(deadline)
        self._wait_for_application_endpoints(deadline)
        self._wait_for_camera_stream(deadline)

        output_at_readiness = ''.join(
            event.text.decode(errors='replace')
            for event in list(proc_output)
        )
        for pattern in (
            rb'gz sim',
            rb'parameter_bridge',
            rb'robot_state_publisher',
            rb'move_group',
            rb'skill_server_node',
            rb'camera_object_provider',
            rb'executive_node',
        ):
            self.assertTrue(
                process_command_matches(pattern),
                f'Required process exited before teardown: {pattern!r}',
            )
        self.assertFalse(
            process_command_matches(rb'(^|/)rviz2?( |$)'),
            'RViz must not run headlessly',
        )
        unexpected_errors = [
            line
            for line in output_at_readiness.splitlines()
            if re.search(r'\b(?:ERROR|FATAL)\b', line)
            and 'No 3D sensor plugin(s) defined for octomap updates' not in line
        ]
        self.assertEqual([], unexpected_errors)

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

    def _wait_for_application_endpoints(self, deadline):
        action_clients = (
            ActionClient(self.node, Home, '/home'),
            ActionClient(self.node, Pick, '/pick'),
            ActionClient(self.node, Place, '/place'),
        )
        service_clients = (
            self.node.create_client(DetectObjects, '/detect_objects'),
            self.node.create_client(SyncObjects, '/sync_objects'),
        )

        try:
            while time.monotonic() < deadline:
                actions_ready = all(
                    client.wait_for_server(timeout_sec=0.1)
                    for client in action_clients
                )
                services_ready = all(
                    client.wait_for_service(timeout_sec=0.1)
                    for client in service_clients
                )
                if actions_ready and services_ready:
                    return
        finally:
            for client in action_clients:
                client.destroy()
            for client in service_clients:
                self.node.destroy_client(client)

        self.fail('The five required application endpoints did not become ready')

    def _wait_for_camera_stream(self, deadline):
        images = []
        camera_infos = []
        image_subscription = self.node.create_subscription(
            Image,
            '/camera/image_raw',
            images.append,
            qos_profile_sensor_data,
        )
        camera_info_subscription = self.node.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            camera_infos.append,
            qos_profile_sensor_data,
        )

        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if images and camera_infos:
                    return
        finally:
            self.node.destroy_subscription(image_subscription)
            self.node.destroy_subscription(camera_info_subscription)

        self.fail('Camera image or camera info did not become ready')
