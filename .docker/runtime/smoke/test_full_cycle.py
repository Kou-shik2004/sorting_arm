"""Check that the application completes one complete sorting cycle."""

import re
import threading
import time
import unittest

import launch
import launch.actions
import launch_testing
import launch_testing.actions
from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image


def stamp_nanoseconds(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class CameraDiagnostics:

    REPORT_INTERVAL_SECONDS = 10.0

    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.started_at = time.monotonic()
        self.last_report_at = self.started_at
        self.counts = {'rgb': 0, 'depth': 0, 'pair': 0}
        self.report_counts = self.counts.copy()
        self.last_arrivals = {'rgb': None, 'depth': None, 'pair': None}
        self.maximum_gaps = {'rgb': 0.0, 'depth': 0.0, 'pair': 0.0}
        self.maximum_pair_stamp_difference = 0.0
        self.latest_sim_nanoseconds = None
        self.report_sim_nanoseconds = None
        self.report_sim_wall_time = None

        self.rgb_subscriber = Subscriber(
            node,
            Image,
            '/camera/image_raw',
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_subscriber = Subscriber(
            node,
            Image,
            '/camera/depth/image_raw',
            qos_profile=qos_profile_sensor_data,
        )
        self.rgb_subscriber.registerCallback(self._record_rgb)
        self.depth_subscriber.registerCallback(self._record_depth)

        self.synchronizer = ApproximateTimeSynchronizer(
            [self.rgb_subscriber, self.depth_subscriber],
            queue_size=5,
            slop=0.010,
        )
        self.synchronizer.registerCallback(self._record_pair)
        self.clock_subscription = node.create_subscription(
            Clock,
            '/clock',
            self._record_clock,
            qos_profile_sensor_data,
        )

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(node)
        self.executor_thread = threading.Thread(
            target=self.executor.spin,
            name='full_cycle_diagnostics',
            daemon=True,
        )
        self.executor_thread.start()

    def stop(self):
        self.executor.shutdown(timeout_sec=5.0)
        self.executor_thread.join(timeout=5.0)
        self.executor.remove_node(self.node)

    def report_if_due(self):
        now = time.monotonic()
        if now - self.last_report_at >= self.REPORT_INTERVAL_SECONDS:
            self._report(now, 'periodic')

    def report_final(self, reason):
        self._report(time.monotonic(), reason)

    def _record_arrival(self, stream):
        now = time.monotonic()
        with self.lock:
            last_arrival = self.last_arrivals[stream]
            if last_arrival is not None:
                self.maximum_gaps[stream] = max(
                    self.maximum_gaps[stream],
                    now - last_arrival,
                )
            self.last_arrivals[stream] = now
            self.counts[stream] += 1

    def _record_rgb(self, _message):
        self._record_arrival('rgb')

    def _record_depth(self, _message):
        self._record_arrival('depth')

    def _record_pair(self, rgb, depth):
        difference = abs(
            stamp_nanoseconds(rgb.header.stamp)
            - stamp_nanoseconds(depth.header.stamp)
        ) / 1_000_000_000.0
        self._record_arrival('pair')
        with self.lock:
            self.maximum_pair_stamp_difference = max(
                self.maximum_pair_stamp_difference,
                difference,
            )

    def _record_clock(self, message):
        now = time.monotonic()
        with self.lock:
            self.latest_sim_nanoseconds = stamp_nanoseconds(message.clock)
            if self.report_sim_nanoseconds is None:
                self.report_sim_nanoseconds = self.latest_sim_nanoseconds
                self.report_sim_wall_time = now

    def _report(self, now, reason):
        with self.lock:
            elapsed = now - self.last_report_at
            rates = {
                stream: (self.counts[stream] - self.report_counts[stream])
                / elapsed
                for stream in self.counts
            }
            current_gaps = {
                stream: (
                    None
                    if self.last_arrivals[stream] is None
                    else now - self.last_arrivals[stream]
                )
                for stream in self.counts
            }
            if (
                self.latest_sim_nanoseconds is None
                or self.report_sim_nanoseconds is None
                or self.report_sim_wall_time is None
                or now <= self.report_sim_wall_time
            ):
                real_time_factor = None
            else:
                real_time_factor = (
                    self.latest_sim_nanoseconds - self.report_sim_nanoseconds
                ) / 1_000_000_000.0 / (now - self.report_sim_wall_time)

            counts = self.counts.copy()
            maximum_gaps = self.maximum_gaps.copy()
            maximum_pair_stamp_difference = (
                self.maximum_pair_stamp_difference
            )
            self.last_report_at = now
            self.report_counts = counts.copy()
            self.report_sim_nanoseconds = self.latest_sim_nanoseconds
            self.report_sim_wall_time = now

        rgb_summary = self._format_stream(
            rates['rgb'],
            counts['rgb'],
            maximum_gaps['rgb'],
            current_gaps['rgb'],
        )
        depth_summary = self._format_stream(
            rates['depth'],
            counts['depth'],
            maximum_gaps['depth'],
            current_gaps['depth'],
        )
        pair_summary = self._format_stream(
            rates['pair'],
            counts['pair'],
            maximum_gaps['pair'],
            current_gaps['pair'],
        )
        print(
            'camera diagnostics '
            f'{reason}: wall={now - self.started_at:.3f}s '
            f'rgb={rgb_summary} '
            f'depth={depth_summary} '
            f'pair={pair_summary} '
            f'pair_stamp_gap_max={maximum_pair_stamp_difference:.6f}s '
            f'rtf={self._format_optional(real_time_factor)}',
            flush=True,
        )

    @staticmethod
    def _format_stream(rate, count, maximum_gap, current_gap):
        return (
            f'{rate:.3f}Hz/count={count}/max_gap={maximum_gap:.3f}s/'
            f'age={CameraDiagnostics._format_optional(current_gap, "s")}'
        )

    @staticmethod
    def _format_optional(value, suffix=''):
        if value is None:
            return 'unavailable'
        return f'{value:.3f}{suffix}'


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
    rgb_render_rate = launch.actions.ExecuteProcess(
        cmd=['gz', 'topic', '--frequency', '--topic', '/camera/image'],
        name='rgb_render_rate',
        output='screen',
    )
    depth_render_rate = launch.actions.ExecuteProcess(
        cmd=[
            'gz',
            'topic',
            '--frequency',
            '--topic',
            '/camera/depth_image',
        ],
        name='depth_render_rate',
        output='screen',
    )

    return (
        launch.LaunchDescription(
            [
                app_process,
                rgb_render_rate,
                depth_render_rate,
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
        self.diagnostics = CameraDiagnostics(self.node)

    def tearDown(self):
        self.diagnostics.stop()
        self.node.destroy_node()

    def test_cycle_completes(self, proc_info, proc_output, app_process):
        proc_info.assertWaitForStartup(app_process, timeout=10)
        deadline = time.monotonic() + 300.0

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
                self.diagnostics.report_final('success')
                self.assertGreater(total_jobs, 0)
                self.assertEqual(total_jobs, completed_jobs)
                return

            failure = re.search(
                r'sorting cycle failed(?: before completion)?:',
                output,
            )
            if failure:
                self.diagnostics.report_final('terminal-failure')
            self.assertIsNone(
                failure,
                'The executive reported a terminal sorting-cycle failure',
            )
            self.diagnostics.report_if_due()
            threading.Event().wait(1.0)

        self.diagnostics.report_final('deadline')
        self.fail('The executive did not report a terminal sorting-cycle result')
