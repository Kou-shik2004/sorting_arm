"""Check that the application completes one complete sorting cycle."""

import re
import time
import unittest

import launch
import launch.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy


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


class TestFullSortingCycle(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('sorting_arm_full_cycle')

    def tearDown(self):
        self.node.destroy_node()

    def test_cycle_completes(self, proc_info, proc_output, app_process):
        proc_info.assertWaitForStartup(app_process, timeout=10)
        deadline = time.monotonic() + 900.0

        while time.monotonic() < deadline:
            output = ''.join(
                event.text.decode(errors='replace')
                for event in list(proc_output)
            )
            success = re.search(
                r'sorting cycle succeeded: completed=(\d+) total=(\d+)',
                output,
            )
            if success:
                completed_jobs, total_jobs = map(int, success.groups())
                self.assertGreater(total_jobs, 0)
                self.assertEqual(total_jobs, completed_jobs)
                return

            self.assertIsNone(
                re.search(r'sorting cycle failed(?: before completion)?:', output),
                'The executive reported a terminal sorting-cycle failure',
            )
            rclpy.spin_once(self.node, timeout_sec=1.0)

        self.fail('The executive did not report a terminal sorting-cycle result')
