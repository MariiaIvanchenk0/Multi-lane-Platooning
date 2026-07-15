import os
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python import get_package_share_directory
from launch.actions import GroupAction, ExecuteProcess

params_config = os.path.join(get_package_share_directory('simulation'), 'config', 'params.yaml')
rviz_config = os.path.join(get_package_share_directory('simulation'), 'config', 'config_sim.rviz')

def generate_launch_description():
    # [robot_id, initial_s, initial_l, initial_v, [neighbor_ids], assigned_lane]
    namespace = "robot"
    platoon_config = [
        [1,  0.0, -0.3, 30.0, [2, 4], 0.0], # [2, 4]
        [2, 22.0, 0.0, 25.0, [1, 3, 5], 0.0], # [1, 3, 5]
        [3, 48.0,  0.5, 26.0, [2, 5], 0.0], # [2, 5]
        [4, 10.0, -3.4, 31.0, [1, 5], 4.0], # [1, 5]
        # [5, 45.0, -4.0, 27.0, [2, 3, 4], 4.0]
    ]

    launch_nodes = []
    for robot_id, s0, l0, v0, neighbors, lane in platoon_config:
        namespace_string = f"{namespace}_{robot_id}"
        
        robot_group = GroupAction([
            PushRosNamespace(namespace_string),
            
            Node(
                package='simulation',
                executable='model_simulation_node',
                parameters=[params_config, {
                    'id': robot_id,
                    's0': s0,
                    'l0': l0,
                    'v0': v0,
                }]
            ),

            Node(
                package='simulation',
                executable='controller_node',
                name='controller',
                parameters=[params_config, {
                    'l_lane': lane,
                    'k_1': 3.0,
                    'k_2': 0.3,
                }]
            ),
            
            # Node(
            #     package='simulation',
            #     executable='lateral_controller_node',
            #     name='lateral_controller',
            #     parameters=[params_config, {
            #         'l_lane': lane,
            #     }]
            # ),
            
            # Node(
            #     package='simulation',
            #     executable='longitudinal_controller_node',
            #     name='longitudinal_controller',
            #     parameters=[params_config, {
            #         'k_1': 3.0,
            #         'k_2': 0.3
            #     }]
            # ),

            # Node(
            #     package='simulation',
            #     executable='road_adaptation_node',
            #     name='road_adaptation',
            #     parameters=[params_config, {
            #         'l0': l0,
            #     }]
            # ),
            
            Node(
                package='simulation',
                executable='formation_controller_node',
                name='formation_controller',
                parameters=[params_config, {
                    'id': robot_id,
                    'neighbor_ids': neighbors,
                    'l0': l0,
                }]
            ),
        ])
        
        launch_nodes.append(robot_group)

    rviz = ExecuteProcess(
        cmd = [
            'ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_config
        ], 
        output = 'screen'
    )
    launch_nodes.append(rviz)

    return LaunchDescription(launch_nodes)