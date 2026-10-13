"""Check menu routing and saved-reference validation without running robot nodes."""

import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import OpaqueFunction, RegisterEventHandler


LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'proxskin_soc_test.launch.py'
SPEC = importlib.util.spec_from_file_location('experiment_menu', LAUNCH_FILE)
MENU = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MENU)


class ExperimentLaunchTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.start_file = Path(self.directory.name) / 'start.json'
        self.start_file.write_text(json.dumps({
            'version': 2, 'start_distance_mm': 300, 'robot_name': 'UR10_right',
            'tcp_pose': [0.4, -0.2, 0.7, 0.0, 0.0, 0.0],
            'joint_rad': [0.2, -1.5, -1.5, -1.5, 1.5, -1.5],
            'orientation_aligned': True,
        }))
        self.context = LaunchContext()
        self.context.launch_configurations.update(
            mode='vertical', start_distance_mm='300', repeat_count='2', dwell_seconds='0.1',
            config_file='', enable_ecan='true', start_file=str(self.start_file),
            x_span_mm='600', spacing_mm='1', x_direction='centered',
        )
        self.kwargs = []
        real_node = MENU.Node

        def capture(**kwargs):
            self.kwargs.append(kwargs)
            return real_node(**kwargs)

        mock = patch.object(MENU, 'Node', side_effect=capture)
        mock.start()
        self.addCleanup(mock.stop)

    def test_experiments_use_start_and_downward_distance(self):
        for mode in ['2', '3', '4']:
            self.context.launch_configurations['mode'] = mode
            MENU._launch_scan(self.context)
            options = self.kwargs[-1]['parameters'][1]
            self.assertTrue(options['experiment_start_enabled'])
            self.assertFalse(options['sensor_zero_enabled'])
            self.assertEqual(options['z_max_offset_mm'], 300.0)
            self.assertEqual(options['repeat_count'], 2)
            self.assertTrue(options['enable_ecan'])
            self.assertEqual(options['experiment_start_joint_rad'][0], 0.2)
        self.assertEqual(self.kwargs[1]['executable'], 'proxskin_test')
        self.assertEqual(self.kwargs[1]['parameters'][1]['x_span_mm'], 600.0)
        self.assertEqual(self.kwargs[2]['parameters'][1]['dwell_seconds'], 0.1)

    def test_setup_bypasses_sensors_and_returns_to_menu(self):
        self.context.launch_configurations['mode'] = 'menu'
        def menu_open(path, *args, **kwargs):
            if path == '/dev/tty':
                return io.StringIO('1\n')
            return open(path, *args, **kwargs)

        with patch.object(MENU, 'open', side_effect=menu_open, create=True):
            actions = MENU._launch_scan(self.context)
        options = self.kwargs[-1]['parameters'][1]
        self.assertTrue(options['start_setup_mode'])
        self.assertFalse(options['experiment_start_enabled'])
        self.assertFalse(options['enable_ecan'])
        self.assertFalse(options['base_mode'])
        self.assertIsInstance(actions[0], RegisterEventHandler)
        event = type('Event', (), {'returncode': 0})()
        self.assertIsInstance(MENU._menu_after_setup_exit(event, self.context)[0], OpaqueFunction)

    def test_all_scan_modes_return_to_menu(self):
        self.context.launch_configurations.update(mode='menu', enable_ecan='false')
        for mode in ['2', '3', '4']:
            def menu_open(path, *args, **kwargs):
                if path == '/dev/tty':
                    return io.StringIO(f'{mode}\n1\n0.1\n')
                return open(path, *args, **kwargs)

            with patch.object(MENU, 'open', side_effect=menu_open, create=True):
                actions = MENU._launch_scan(self.context)
            self.assertIsInstance(actions[0], RegisterEventHandler)
            self.assertTrue(self.kwargs[-1]['parameters'][1]['experiment_start_enabled'])
            self.assertFalse(self.kwargs[-1]['parameters'][1]['enable_ecan'])
        event = type('Event', (), {'returncode': 1})()
        self.assertIsInstance(MENU._menu_after_setup_exit(event, self.context)[0], OpaqueFunction)

    def test_automated_modes_do_not_open_menu(self):
        actions = MENU._launch_scan(self.context)
        self.assertEqual(len(actions), 1)
        self.assertNotIsInstance(actions[0], RegisterEventHandler)

    def test_missing_start_blocks_experiments_but_allows_capture(self):
        self.start_file.unlink()
        with self.assertRaises(RuntimeError):
            MENU._launch_scan(self.context)
        self.assertEqual(len(self.kwargs), 0)
        self.context.launch_configurations['mode'] = '1'
        MENU._launch_scan(self.context)
        self.assertTrue(self.kwargs[-1]['parameters'][1]['start_setup_mode'])

    def test_corrupt_reference_and_invalid_distances(self):
        for value in ['0', '-1', 'nan', 'inf', '-inf', '', 'abc', '200mm']:
            with self.assertRaises(ValueError):
                MENU._parse_start_distance(value)
        self.start_file.write_text('{"version": 1, "robot_name": "UR10_right"}')
        with self.assertRaises(ValueError):
            MENU._load_experiment_start(self.start_file)

    def test_saved_distance_drives_all_experiments(self):
        data = json.loads(self.start_file.read_text())
        for distance in [100, 200, 301, 125.5]:
            data['start_distance_mm'] = distance
            self.start_file.write_text(json.dumps(data))
            for mode in ['2', '3', '4']:
                self.context.launch_configurations['mode'] = mode
                MENU._launch_scan(self.context)
                self.assertEqual(self.kwargs[-1]['parameters'][1]['z_max_offset_mm'], distance)

    def test_old_start_must_be_saved_with_distance(self):
        data = json.loads(self.start_file.read_text())
        data['version'] = 1
        data.pop('start_distance_mm')
        self.start_file.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'Choose 1'):
            MENU._load_experiment_start(self.start_file)

    def test_capture_distance_and_removed_mode_five(self):
        self.context.launch_configurations.update(mode='capture', start_distance_mm='125.5')
        MENU._launch_scan(self.context)
        self.assertEqual(self.kwargs[-1]['parameters'][1]['start_distance_mm'], 125.5)
        self.assertTrue(self.kwargs[-1]['parameters'][1]['capture_start_mode'])
        with self.assertRaises(ValueError):
            MENU._parse_mode('5')

    def test_old_unaligned_start_must_be_aligned_again(self):
        data = json.loads(self.start_file.read_text())
        del data['orientation_aligned']
        self.start_file.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'orientation alignment'):
            MENU._load_experiment_start(self.start_file)

    def test_base_retains_initial_pose_without_changing_start_file(self):
        contents = self.start_file.read_bytes()
        self.context.launch_configurations['mode'] = '0'
        MENU._launch_scan(self.context)
        options = self.kwargs[-1]['parameters'][1]
        self.assertTrue(options['base_mode'])
        self.assertTrue(options['base_hold_j6'])
        self.assertFalse(options['experiment_start_enabled'])
        self.assertFalse(options['enable_ecan'])
        self.assertEqual(self.start_file.read_bytes(), contents)


if __name__ == '__main__':
    unittest.main()
