#!/usr/bin/env python3
"""UR10 base, start setup and motion experiments."""

import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _read_choice(tty, label, default, parser):
    while True:
        suffix = f' [{default}]' if default is not None else ''
        print(f'{label}{suffix}: ', end='', flush=True)
        answer = tty.readline()
        if not answer:
            raise RuntimeError('Input closed. Select the mode with launch arguments.')
        try:
            value = answer.strip()
            if not value and default is not None:
                value = str(default)
            return parser(value)
        except ValueError as exc:
            print(f'Invalid input: {exc}\n', flush=True)


def _parse_mode(value):
    modes = {'0': 'base', '1': 'setup', '2': 'vertical', '3': 'columns', '4': 'stepped',
             'base': 'base', 'setup': 'setup', 'capture': 'capture',
             'vertical': 'vertical', 'columns': 'columns', 'stepped': 'stepped'}
    if value not in modes:
        raise ValueError('Choose a mode from 0 to 4.')
    return modes[value]


def _parse_start_distance(value):
    distance = float(value)
    if not math.isfinite(distance) or distance <= 0.0:
        raise ValueError('Start distance must be a positive number in mm.')
    return distance


def _parse_repetitions(value):
    if not value.strip():
        raise ValueError('Enter round trips, or set repeat_count:=N.')
    try:
        repetitions = int(value)
    except ValueError as exc:
        raise ValueError('Round trips must be a whole number from 1 to 100.') from exc
    if not 1 <= repetitions <= 100:
        raise ValueError('Round trips must be from 1 to 100.')
    return repetitions


def _parse_dwell(value):
    seconds = float(value)
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise ValueError('Hold time must be a positive number of seconds.')
    return seconds


def _parse_enable_ecan(value):
    value = value.strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise ValueError('Set enable_ecan to true or false.')


def _load_experiment_start(path):
    with open(path, encoding='utf-8') as file:
        data = json.load(file)
    if not isinstance(data, dict) or data.get('version') != 2 or not isinstance(data.get('robot_name'), str):
        raise ValueError('Start pose needs a saved distance reference. Choose 1 to set it up again.')
    if data.get('orientation_aligned') is not True:
        raise ValueError('Start pose needs orientation alignment. Choose 1 to set it up again.')
    distance = data.get('start_distance_mm')
    if isinstance(distance, bool) or not isinstance(distance, (int, float)):
        raise ValueError('Missing start distance. Choose 1 to set up the start.')
    _parse_start_distance(distance)
    for field in ('tcp_pose', 'joint_rad'):
        values = data.get(field)
        if not isinstance(values, list) or len(values) != 6 or any(
                isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                for value in values):
            raise ValueError(f'Invalid {field} in the start pose file.')
    if any(abs(value) > 2 * math.pi for value in data['joint_rad']):
        raise ValueError('Saved joint angles are outside the allowed range.')
    return data


def _positive_mm(value, label, allow_zero=False):
    number = float(value)
    if not math.isfinite(number) or number < 0.0 or (number == 0.0 and not allow_zero):
        raise ValueError(f'Invalid {label}.')
    return number


def _menu_after_setup_exit(event, context):
    if event.returncode >= 0 and not context.is_shutdown:
        print('\nReturning to menu.\n', flush=True)
        return [OpaqueFunction(function=_launch_scan)]
    return []


def _launch_scan(context):
    mode = LaunchConfiguration('mode').perform(context)
    interactive = mode == 'menu'
    enable_ecan = _parse_enable_ecan(LaunchConfiguration('enable_ecan').perform(context))
    start_file = os.path.abspath(os.path.expanduser(LaunchConfiguration('start_file').perform(context)))
    scans = ('vertical', 'columns', 'stepped')
    start = None
    if interactive:
        try:
            saved = _load_experiment_start(start_file)
            status = f"Saved ({saved['start_distance_mm']:g} mm)"
        except (OSError, ValueError):
            status = 'Choose 1 to set up the start'
        print(
            '\nUR10 Motion Test\n'
            '----------------\n'
            '  0  Move to base pose\n'
            '  1  Set up start position and distance\n'
            '\n'
            '  2  Z down / up\n'
            '  3  X-Z scan (Y fixed)\n'
            '  4  Z down / up with 1 cm stops\n'
            '\n'
            f'Start:   {status}\n'
            f'Sensors: {"ON" if enable_ecan else "OFF (sensor column empty)"}\n'
            'CSV:     100 Hz | time (s), robot distance (mm), CAN 0x41 front raw\n', flush=True)
        try:
            with open('/dev/tty', encoding='utf-8') as tty:
                mode = _read_choice(tty, 'Choose mode', None, _parse_mode)
                if mode in scans:
                    try:
                        start = _load_experiment_start(start_file)
                    except (OSError, ValueError) as exc:
                        print(f'\nCannot load start: {exc}\n', flush=True)
                        return [OpaqueFunction(function=_launch_scan)]
                    repetitions = _read_choice(tty, 'Round trips per X position (1-100)' if mode == 'columns'
                                               else 'Round trips (1-100)', None, _parse_repetitions)
                    dwell = _read_choice(tty, 'Hold time at each stop (s)',
                                         LaunchConfiguration('dwell_seconds').perform(context), _parse_dwell) if mode == 'stepped' else 0.0
        except OSError as exc:
            raise RuntimeError('No interactive terminal. Set mode:=base/vertical/columns/stepped/capture.') from exc
    else:
        mode = _parse_mode(mode)
        if mode in scans:
            try:
                start = _load_experiment_start(start_file)
            except (OSError, ValueError) as exc:
                raise RuntimeError(f'Cannot load start: {exc}\nChoose 1 to set up the start first.') from exc
            repetitions = _parse_repetitions(LaunchConfiguration('repeat_count').perform(context))
            dwell = _parse_dwell(LaunchConfiguration('dwell_seconds').perform(context)) if mode == 'stepped' else 0.0

    package_dir = get_package_share_directory('ur10_proxskin_test')
    config = LaunchConfiguration('config_file').perform(context)
    if not config:
        config = os.path.join(package_dir, 'config', 'proxskin_test.yaml' if mode == 'columns' else 'proxskin_vertical_test.yaml')
    overrides = {
        'base_mode': mode == 'base',
        'base_hold_j6': True,
        'start_setup_mode': mode == 'setup',
        'capture_start_mode': mode == 'capture',
        'start_file': start_file,
        'calibration_mode': False,
        'sensor_zero_enabled': False,
        'experiment_start_enabled': mode in scans,
        'enable_ecan': enable_ecan and mode in scans,
        'initial_pose_enabled': True,
        'initial_arm_joint_deg': [-90.0, -90.0, -90.0, 90.0, -90.0],
    }
    if mode == 'base':
        print('\nMoving to base pose. J1 and J6 stay fixed.\n', flush=True)
    elif mode == 'setup':
        print('\nStart setup: choose current position, or adjust Z (a: align, e: return).\n', flush=True)
    elif mode == 'capture':
        distance = _parse_start_distance(LaunchConfiguration('start_distance_mm').perform(context))
        overrides['start_distance_mm'] = distance
        print(f'\nAligning tool Z at the current XYZ with J6 held, and saving a {distance:g} mm start.\n', flush=True)
    else:
        distance = float(start['start_distance_mm'])
        overrides.update({
            'experiment_start_robot_name': start['robot_name'],
            'experiment_start_pose': [float(v) for v in start['tcp_pose']],
            'experiment_start_joint_rad': [float(v) for v in start['joint_rad']],
            'z_max_offset_mm': distance,
            'repeat_count': repetitions,
            'dwell_seconds': dwell,
        })
        names = {'vertical': 'Z down / up', 'columns': 'X-Z scan (Y fixed)', 'stepped': 'Z down / up with stops'}
        summary = [f'\nStarting: {names[mode]}', f'  Distance: {distance:.2f} -> 0.00 -> {distance:.2f} mm',
                   f'  Round trips: {repetitions}' + (' per X position' if mode == 'columns' else '')]
        if mode == 'stepped':
            summary.append(f'  Stops: every 10 mm, hold {dwell:g} s')
        if mode == 'columns':
            span = _positive_mm(LaunchConfiguration('x_span_mm').perform(context), 'X span', allow_zero=True)
            spacing = _positive_mm(LaunchConfiguration('spacing_mm').perform(context), 'X spacing')
            direction = LaunchConfiguration('x_direction').perform(context)
            if direction not in ('centered', 'positive', 'negative'):
                raise ValueError('Set x_direction to centered, positive, or negative.')
            overrides.update({'use_relative_vertical_range': True, 'use_fixed_orientation': False,
                              'fix_wrist_joints': False, 'x_offset_mm': 0.0,
                              'x_span_mm': span, 'spacing_mm': spacing, 'x_direction': direction})
            first, last = {'centered': (span / 2, -span / 2), 'positive': (0, span), 'negative': (0, -span)}[direction]
            summary.extend([f'  X offsets: {first:+g} -> {last:+g} mm from start',
                            f'  X step: {spacing:g} mm ({math.ceil(span / spacing) + 1} positions)'])
        summary.extend(['  Endpoints: wait for measured arrival before reversing',
                        '  CSV: 100 Hz | time (s), robot distance (mm), CAN 0x41 front raw',
                        '  Cancel: q stops the experiment, returns to the saved start, then opens the menu',
                        '  Press q again during the return to stop it (no Enter needed)'])
        print('\n'.join(summary) + '\n', flush=True)

    executable = 'proxskin_test' if mode == 'columns' else 'proxskin_vertical_test'
    node = Node(package='ur10_proxskin_test', executable=executable, name=executable,
                output='screen', output_format='{line}', emulate_tty=True,
                parameters=[config, overrides])
    if interactive:
        return [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=_menu_after_setup_exit)), node]
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='menu', description='menu, base(0), setup(1), vertical(2), columns(3), stepped(4); capture for automated start saving'),
        DeclareLaunchArgument('start_file', default_value='/home/ur/.ros/ur10_proxskin_test/experiment_start.json', description='Aligned pose and positive distance in mm saved by mode 1'),
        DeclareLaunchArgument('start_distance_mm', default_value='300', description='Positive distance in mm; used only by automated mode:=capture'),
        DeclareLaunchArgument('enable_ecan', default_value='true', description='true: connect to eCAN and record CAN 0x41 front raw; false: run without sensors'),
        DeclareLaunchArgument('x_span_mm', default_value='600', description='Total X range in mm'),
        DeclareLaunchArgument('spacing_mm', default_value='1', description='X column spacing in mm'),
        DeclareLaunchArgument('x_direction', default_value='centered', description='centered: +span/2 to -span/2; positive: +X; negative: -X (UR base)'),
        DeclareLaunchArgument('repeat_count', default_value='', description='Required round trips (1..100) for noninteractive scans'),
        DeclareLaunchArgument('dwell_seconds', default_value='2', description='Hold time at each 10 mm stop in mode 4'),
        DeclareLaunchArgument('config_file', default_value='', description='Optional node parameter YAML'),
        OpaqueFunction(function=_launch_scan),
    ])
