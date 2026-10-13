#!/usr/bin/env python3
"""
Launch proxskin_test.

Note: urcb2_driver's `single_arm` executable currently has a hardcoded robot IP
and is not parameterized for launch files, so it must be started separately:
  ros2 run urcb2_driver single_arm

Usage:
  ros2 launch ur10_proxskin_test proxskin_test.launch.py
  ros2 launch ur10_proxskin_test proxskin_test.launch.py config_file:=/path/to/proxskin_test.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('ur10_proxskin_test')
    default_config_file = os.path.join(package_dir, 'config', 'proxskin_test.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='ROS 2 parameter YAML file for proxskin_test'
    )

    proxskin_test_node = Node(
        package='ur10_proxskin_test',
        executable='proxskin_test',
        name='proxskin_test',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_file_arg,
        proxskin_test_node,
    ])
