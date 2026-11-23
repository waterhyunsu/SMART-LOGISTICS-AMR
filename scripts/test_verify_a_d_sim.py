"""Offline checks. These tests never launch ROS 2 or Gazebo."""
import json
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from verify_a_d_sim import GoalEvidence, load_config, ros_context, run_pr14_d, verify_evidence


UUID = '0123456789abcdef0123456789abcdef'
OTHER_UUID = 'f' * 32
POSE = (1.0, -0.5, 0.3)
SEND = f'[INFO] [D]: [SEND] {UUID} → waypoint TEST (1.0, -0.5, 0.3)'
SCRIPT = Path(__file__).with_name('verify_a_d_sim.py')


class RosContextTests(unittest.TestCase):
    def test_init_return_value_is_ignored_and_shutdown_follows_work(self):
        ros = Mock()
        ros.init.return_value = None  # ROS 2 Jazzy
        args = ['--ros-args', '-p', 'use_sim_time:=true']
        with ros_context(ros, args):
            ros.spin()
        self.assertEqual(ros.mock_calls, [call.init(args=args), call.spin(), call.try_shutdown()])

    def test_shutdown_runs_when_work_raises(self):
        ros = Mock()
        ros.init.return_value = None
        with self.assertRaisesRegex(RuntimeError, 'startup failed'):
            with ros_context(ros, []):
                raise RuntimeError('startup failed')
        ros.try_shutdown.assert_called_once_with()

    def test_failed_init_propagates(self):
        ros = Mock()
        ros.init.side_effect = RuntimeError('init failed')
        with self.assertRaisesRegex(RuntimeError, 'init failed'):
            with ros_context(ros, []):
                self.fail('entered context after failed init')
        ros.try_shutdown.assert_not_called()

    def test_builtin_d_waits_for_nav2_and_cleans_up_on_shutdown(self):
        class ExternalShutdown(Exception):
            pass

        for stop in (KeyboardInterrupt, ExternalShutdown):
            with self.subTest(stop=stop):
                ros = Mock()
                ros.spin.side_effect = stop
                d_adapter = Mock()
                config = SimpleNamespace(waypoint_id='TEST', waypoint_pose=POSE)
                with patch.dict(sys.modules, {
                    'rclpy': ros,
                    'rclpy.executors': SimpleNamespace(ExternalShutdownException=ExternalShutdown),
                    'amr_navigation': SimpleNamespace(gate1_mock=d_adapter),
                }):
                    run_pr14_d(config)

                ros.init.assert_called_once_with(args=[
                    '--ros-args', '-p', 'use_sim_time:=true',
                    '-p', 'wait_for_nav2_before_dispatch:=true',
                ])
                d_adapter.DNavigationNode.return_value.destroy_node.assert_called_once_with()
                ros.try_shutdown.assert_called_once_with()


class EvidenceTests(unittest.TestCase):
    def check(self, *, log=SEND, statuses=None, waiting=True, position=(1.1, -0.5)):
        evidence = GoalEvidence(UUID, log, {UUID: 4} if statuses is None else statuses,
                                waiting, position)
        return verify_evidence(evidence, 'TEST', POSE, 0.25)

    def test_correlates_send_success_and_handoff(self):
        self.assertAlmostEqual(self.check(), 0.1)

    def test_rejects_repeat_send_even_when_uuid_matches(self):
        with self.assertRaisesRegex(ValueError, 'exactly one'):
            self.check(log=SEND + '\n' + SEND)

    def test_rejects_missing_or_mismatched_d_uuid(self):
        with self.assertRaisesRegex(ValueError, 'exactly one'):
            self.check(log='')
        with self.assertRaisesRegex(ValueError, 'UUID'):
            self.check(log=SEND.replace(UUID, OTHER_UUID))

    def test_rejects_mismatched_status_and_unsuccessful_status(self):
        for statuses in ({}, {OTHER_UUID: 4}, {UUID: 6}, {UUID: 4, OTHER_UUID: 4}):
            with self.subTest(statuses=statuses), self.assertRaisesRegex(ValueError, 'Nav2'):
                self.check(statuses=statuses)

    def test_rejects_d_waypoint_or_pose_mismatch(self):
        for log in (SEND.replace('TEST', 'OTHER'), SEND.replace('1.0', '1.1')):
            with self.subTest(log=log), self.assertRaisesRegex(ValueError, 'Waypoint'):
                self.check(log=log)

    def test_requires_handoff_wait_and_goal_proximity(self):
        with self.assertRaisesRegex(ValueError, 'waiting_confirmation'):
            self.check(waiting=False)
        with self.assertRaisesRegex(ValueError, 'position'):
            self.check(position=(2.0, -0.5))


class ConfigureTests(unittest.TestCase):
    def setUp(self):
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.path = self.directory / 'config.json'
        (self.directory / 'map.yaml').touch()
        self.config = {
            'sim_command': ['ros2', 'launch', 'team_bringup', 'sim.launch.py', 'map:={map_yaml}'],
            'map_yaml': 'map.yaml',
            'ros_domain_id': 94,
            'initial_pose': [0.0, 0.0, 0.0],
            'waypoint': {'id': 'TEST', 'pose': list(POSE)},
        }

    def write(self):
        self.path.write_text(json.dumps(self.config))
        return self.path

    def test_expands_map_relative_to_config_file(self):
        config = load_config(self.write())
        self.assertIn(f'map:={self.directory / "map.yaml"}', config.sim_command)

    def test_rejects_nonfinite_pose_invalid_domain_and_missing_map(self):
        for patch in ({'initial_pose': [0, 0, float('nan')]},
                      {'ros_domain_id': 0}, {'map_yaml': 'missing.yaml'},
                      {'initial_pose': list(POSE)}):
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                original = self.config
                self.config = original | patch
                try:
                    load_config(self.write())
                finally:
                    self.config = original

    def test_default_execution_only_validates_configuration(self):
        self.config['sim_command'][0] = 'this-executable-does-not-exist'
        result = subprocess.run([sys.executable, str(SCRIPT), str(self.write())],
                                capture_output=True, text=True, check=True)
        self.assertIn('configuration valid', result.stdout)
        self.assertNotIn('PASS', result.stdout)
