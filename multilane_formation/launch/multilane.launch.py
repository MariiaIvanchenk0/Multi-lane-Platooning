import os
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


params_config = os.path.join(get_package_share_directory('multilane_formation'), 'config', 'params.yaml')
# rviz_config = os.path.join(get_package_share_directory('multilane_formation'), 'config', 'config_sim.rviz')
rviz_config = os.path.join(get_package_share_directory('multilane_formation'), 'config', 'config2.rviz')


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz and conditional nodes.'
    )

    use_rviz = LaunchConfiguration('use_rviz')

    # [robot_id, initial_s, initial_l, initial_v, [neighbor_ids], assigned_lane]
    namespace = "robot"
    platoon_config = [
        [1, 0.0, 0.0, 0.5, [2], 0.0],
        [2, 1.2, 0.0, 0.5, [1], 0.0],
    ]

    assigned_lanes = [float(config[5]) for config in platoon_config]
    unique_lanes = list(set(assigned_lanes + [0.0]))

    launch_nodes = [
        use_rviz_arg,
    ]
    
    for robot_id, s0, l0, v0, neighbors, lane in platoon_config:
        namespace_string = f"{namespace}_{robot_id}"
        
        robot_group = GroupAction([
            PushRosNamespace(namespace_string),

            Node(
                package='multilane_formation',
                executable='controller_node',
                name='controller',
                parameters=[params_config, {
                    'l_lane': lane,
                    'k_1': 3.0,
                    'k_2': 0.3,
                    'id': robot_id,
                    's0': s0,
                    'l0': l0,
                    'v0': v0,
                    'viz_lanes': unique_lanes,
                }]
            ),

            Node(
                package='multilane_formation',
                executable='formation_controller_node',
                name='formation_controller',
                parameters=[params_config, {
                    'id': robot_id,
                    'neighbor_ids': neighbors,
                    'l0': l0,
                    'namespace': namespace,  # explicit so neighbor topics are always correct
                }]
            ),
        ])
        
        launch_nodes.append(robot_group)

    # ros2 launch vrpn_mocap client.launch.yaml server:=129.97.71.49 port:=3883
    remaps = [SetRemap(src=f'/vrpn_mocap/yahboom_{config[0]}/pose', dst=f'/robot_{config[0]}/pose') for config in platoon_config]
    vrpn_mocap = GroupAction ([
        *remaps,
        IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('vrpn_mocap'),
                'launch',
                'client.launch.yaml',
            ])
        ),
        launch_arguments={'server':'129.97.71.49', 'port' : '3883'}.items(),
    )])
    launch_nodes.append(vrpn_mocap)
    
    #  ros2 launch polytope_safety_net polytope_safety.launch.py boundary_file:=src/polytope_safety_net/config/safety_net_config.yaml  agents_file:=src/polytope_safety_net/config/yahbooms.yaml 
    polytope_safety_net = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('polytope_safety_net'),
                'launch',
                'polytope_safety.launch.py',
            ])
        ),
        launch_arguments={'boundary_file':'src/polytope_safety_net/config/safety_net_config.yaml', 
                            'agents_file' :'src/polytope_safety_net/config/yahbooms.yaml'}.items() 
    )
    launch_nodes.append(polytope_safety_net)

    rviz = ExecuteProcess(
        cmd = [
            'ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_config
        ], 
        output = 'screen',
        condition=IfCondition(use_rviz)
    )
    launch_nodes.append(rviz)

    return LaunchDescription(launch_nodes)