import os
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess

params_config = os.path.join(get_package_share_directory('multilane_formation'), 'config', 'params.yaml')
rviz_config = os.path.join(get_package_share_directory('multilane_formation'), 'config', 'config_sim.rviz')

def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz and conditional nodes.'
    )

    use_rviz = LaunchConfiguration('use_rviz')

    # [robot_id, initial_s, initial_l, initial_v, [neighbor_ids], assigned_lane]
    # Must match the agent names in omnisim/polytope_safety_net's agents.yaml
    # (e.g. agent_1, agent_2, ...) so pose/cmd_vel topics actually connect.
    namespace = "agent"
    platoon_config = [
        [1,   0.0, 0.0, 0.5, [2], 0.0],
        [2,  1.2, 0.0, 0.5, [1], 0.0],
        # [3, -10.0, 2.0, 15.0, [1, 2], 2.0],
    ]
    # platoon_config = [
    #     [1,  0.0, -0.3, 30.0, [2, 4], 0.0], # [2, 4]
    #     [2, 22.0, 0.0, 25.0, [1, 3, 5], 0.0], # [1, 3, 5]
    #     [3, 48.0,  0.5, 26.0, [2, 5], 0.0], # [2, 5]
    #     [4, 10.0, -3.4, 31.0, [1, 5], 4.0], # [1, 5]
    #     [5, 45.0, -4.0, 27.0, [2, 3, 4], 4.0]
    # ]

    assigned_lanes = [float(config[5]) for config in platoon_config]
    unique_lanes = list(set(assigned_lanes + [0.0]))

    launch_nodes = [
        use_rviz_arg,
    ]
    
    for robot_id, s0, l0, v0, neighbors, lane in platoon_config:
        namespace_string = f"{namespace}_{robot_id}"
        
        robot_group = GroupAction([
            PushRosNamespace(namespace_string),
            
            # Node(
            #     package='multilane_formation',
            #     executable='model_simulation_node',
            #     parameters=[params_config, {
            #         'id': robot_id,
            #         's0': s0,
            #         'l0': l0,
            #         'v0': v0,
            #         'viz_lanes': unique_lanes,
            #     }]
            # ),

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

    rviz = ExecuteProcess(
        cmd = [
            'ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_config
        ], 
        output = 'screen',
        condition=IfCondition(use_rviz)
    )
    launch_nodes.append(rviz)

    return LaunchDescription(launch_nodes)