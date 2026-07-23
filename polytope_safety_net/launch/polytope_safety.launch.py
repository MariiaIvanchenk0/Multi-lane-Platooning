#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'polytope_safety_net'
    pkg_share = get_package_share_directory(pkg_name)

    default_agents_file = os.path.join(pkg_share, 'config', 'agents.yaml')
    default_boundary_file = os.path.join(pkg_share, 'config', 'boundary.yaml')

    agents_file = LaunchConfiguration('agents_file')
    boundary_file = LaunchConfiguration('boundary_file')
    boundary_epsilon = LaunchConfiguration('boundary_epsilon')

    safety_node = Node(
        package=pkg_name,
        executable='polytope_safety_node',
        name='polytope_safety_node',
        output='screen',
        parameters=[
            {
                'agents_file': agents_file,
                'boundary_file': boundary_file,
                'boundary_epsilon': boundary_epsilon,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'agents_file',
            default_value=default_agents_file,
            description='Path to the shared agents YAML file.',
        ),
        DeclareLaunchArgument(
            'boundary_file',
            default_value=default_boundary_file,
            description='Path to the safety boundary YAML file.',
        ),
        DeclareLaunchArgument(
            'boundary_epsilon',
            default_value='0.05',
            description='Distance from boundary where outward motion is blocked.',
        ),
        safety_node,
    ])