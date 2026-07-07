import os
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python import get_package_share_directory
from launch.actions import GroupAction, ExecuteProcess

params_config = os.path.join(get_package_share_directory('simulation'), 'config', 'params.yaml')
rviz_config = os.path.join(get_package_share_directory('simulation'), 'config', 'config_sim.rviz')

def generate_launch_description():
    # [robot_id, initial_s, initial_l, [neighbor_ids]]
    namespace = "robot"
    platoon_config = [
        [1,  0.0, -0.3, [2, 4]],
        [2, 22.0,  0.0, [1, 3, 5]],
        [3, 48.0,  0.5, [2, 5]],
        [4, 10.0, -3.4, [1, 5]],
        [5, 45.0, -4.0, [2, 3, 4]]
    ]

    launch_nodes = []
    for robot_id, s0, l0, neighbors in platoon_config:
        namespace_string = f"{namespace}_{robot_id}"
        
        robot_group = GroupAction([
            PushRosNamespace(namespace_string),
            
            Node(
                package='simulation',
                executable='model_simulation_node',
                name='model_simulation',
                parameters=[{
                    'id': robot_id,
                    's0': s0,
                    'l0': l0,
                    'v0': 25.0,
                    'frequency': 20.0
                }, params_config]
            ),
            
            Node(
                package='simulation',
                executable='lateral_controller_node',
                name='lateral_controller',
                parameters=[{
                    'k_a1': 1.0,
                    'k_a2': 2.0
                }, params_config]
            ),
            
            Node(
                package='simulation',
                executable='longitudinal_controller_node',
                name='longitudinal_controller',
                parameters=[{
                    'k_1': 30,
                    'k_2': 0.3
                }, params_config]
            ),
            
            Node(
                package='simulation',
                executable='formation_controller_node',
                name='formation_controller',
                parameters=[{
                    'id': robot_id,
                    'neighbor_ids': neighbors,
                    'v_f': 25.0
                }, params_config]
            ),
        ])

        rviz = ExecuteProcess(
            cmd = [
                'ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_config
            ], 
            output = 'screen'
        )
        
        launch_nodes.append(robot_group)
        launch_nodes.append(rviz)

    return LaunchDescription(launch_nodes)