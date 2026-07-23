#!/usr/bin/env python3

from copy import deepcopy
import os
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _load_agent_names(agents_file):
    path = os.path.expandvars(os.path.expanduser(agents_file))

    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}

    raw_agents = data.get('agents', [])
    if not isinstance(raw_agents, list) or not raw_agents:
        raise RuntimeError(f"No agents found in {path}")

    names = []
    for agent in raw_agents:
        if isinstance(agent, dict):
            if 'name' not in agent:
                raise RuntimeError(f"Agent entry is missing 'name': {agent}")
            names.append(str(agent['name']))
        else:
            names.append(str(agent))

    return names


def _make_pose_display(template, agent_name, index):
    display = deepcopy(template)
    display['Name'] = f'Pose {agent_name}'
    display['Topic']['Value'] = f'/{agent_name}/pose'

    colors = [
        '255; 25; 0',
        '25; 180; 255',
        '80; 220; 80',
        '255; 210; 50',
        '210; 80; 255',
        '255; 130; 40',
        '80; 255; 210',
        '230; 230; 230',
    ]
    display['Color'] = colors[index % len(colors)]
    display['Alpha'] = 1
    display['Enabled'] = True
    display['Value'] = True

    return display


def _configure_rviz(context):
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    if not _as_bool(use_rviz):
        return []

    agents_file = LaunchConfiguration('agents_file').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)

    agent_names = _load_agent_names(agents_file)

    rviz_config = os.path.expandvars(os.path.expanduser(rviz_config))
    with open(rviz_config, 'r') as f:
        rviz_data = yaml.safe_load(f) or {}

    manager = rviz_data.setdefault('Visualization Manager', {})
    displays = manager.setdefault('Displays', [])

    pose_template = next(
        (
            display for display in displays
            if display.get('Class') == 'rviz_default_plugins/Pose'
        ),
        None,
    )

    if pose_template is None:
        raise RuntimeError(f"No rviz_default_plugins/Pose display found in {rviz_config}")

    non_pose_displays = [
        display for display in displays
        if display.get('Class') != 'rviz_default_plugins/Pose'
    ]
    pose_displays = [
        _make_pose_display(pose_template, name, index)
        for index, name in enumerate(agent_names)
    ]
    manager['Displays'] = non_pose_displays + pose_displays

    global_options = manager.setdefault('Global Options', {})
    global_options['Fixed Frame'] = frame_id

    for display in manager['Displays']:
        if 'Reference Frame' in display:
            display['Reference Frame'] = frame_id

    views = manager.get('Views', {})
    current_view = views.get('Current', {})
    if isinstance(current_view, dict):
        current_view['Target Frame'] = frame_id

    panels = rviz_data.get('Panels', [])
    for panel in panels:
        if panel.get('Class') != 'rviz_common/Displays':
            continue

        tree = panel.get('Property Tree Widget', {})
        expanded = tree.get('Expanded')
        if isinstance(expanded, list):
            tree['Expanded'] = [
                item for item in expanded
                if not str(item).startswith('/Pose')
            ]
            tree['Expanded'].extend(
                f'/Pose {name}{index + 1}'
                for index, name in enumerate(agent_names)
            )

    generated_path = os.path.join(
        tempfile.gettempdir(),
        f'omnisim_{os.getpid()}_generated.rviz',
    )

    with open(generated_path, 'w') as f:
        yaml.safe_dump(rviz_data, f, sort_keys=False)

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', generated_path],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ]


def generate_launch_description():
    pkg_name = 'omnisim'
    safety_pkg_name = 'polytope_safety_net'

    agents_file = LaunchConfiguration('agents_file')
    rviz_config = LaunchConfiguration('rviz_config')
    boundary_file = LaunchConfiguration('boundary_file')
    boundary_epsilon = LaunchConfiguration('boundary_epsilon')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    frame_id = LaunchConfiguration('frame_id')
    cmd_timeout = LaunchConfiguration('cmd_timeout')
    initial_spacing = LaunchConfiguration('initial_spacing')
    use_body_frame_cmd = LaunchConfiguration('use_body_frame_cmd')

    safety_net_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(safety_pkg_name),
                'launch',
                'polytope_safety.launch.py',
            ])
        ),
        launch_arguments={
            'agents_file': agents_file,
            'boundary_file': boundary_file,
            'boundary_epsilon': boundary_epsilon,
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_safety_net')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'agents_file',
            default_value=PathJoinSubstitution([
                FindPackageShare(pkg_name),
                'config',
                'agents.yaml',
            ]),
            description='Path to shared agents.yaml.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare(pkg_name),
                'config',
                'config.rviz',
            ]),
            description='Template RViz config. Pose displays are generated from agents_file.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz with generated Pose displays for all agents.',
        ),
        DeclareLaunchArgument(
            'use_safety_net',
            default_value='false',
            description='Start polytope_safety_net with the same agents_file.',
        ),
        DeclareLaunchArgument(
            'boundary_file',
            default_value=PathJoinSubstitution([
                FindPackageShare(safety_pkg_name),
                'config',
                'robohub.yaml',
            ]),
            description='Path to the polytope safety boundary YAML file.',
        ),
        DeclareLaunchArgument(
            'boundary_epsilon',
            default_value='0.05',
            description='Distance from boundary where outward motion is blocked.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='100.0',
            description='Simulation and pose publishing rate in Hz.',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='world',
            description='PoseStamped header frame_id and RViz fixed frame.',
        ),
        DeclareLaunchArgument(
            'cmd_timeout',
            default_value='0.5',
            description='Seconds before stale cmd_vel is treated as zero.',
        ),
        DeclareLaunchArgument(
            'initial_spacing',
            default_value='1.0',
            description='Default spacing for agents without initial_pose in agents.yaml.',
        ),
        DeclareLaunchArgument(
            'use_body_frame_cmd',
            default_value='true',
            description='If true, cmd_vel linear x/y are interpreted in robot body frame.',
        ),
        Node(
            package=pkg_name,
            executable='omni_sim_node',
            name='omni_sim_node',
            output='screen',
            parameters=[{
                'agents_file': agents_file,
                'publish_rate_hz': publish_rate_hz,
                'frame_id': frame_id,
                'cmd_timeout': cmd_timeout,
                'initial_spacing': initial_spacing,
                'use_body_frame_cmd': use_body_frame_cmd,
            }],
        ),
        safety_net_launch,
        OpaqueFunction(function=_configure_rviz),
    ])
