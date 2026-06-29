import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python import get_package_share_directory


params_config = os.path.join(get_package_share_directory('simulation'), 'config', 'params.yaml')
rviz_config = os.path.join(get_package_share_directory('simulation'), 'config', 'config_sim.rviz')

def generate_launch_description():
    model_sim = Node(
        name='model_simulation',
        package='simulation',
        executable='model_simulation_node',
        parameters=[params_config],
    )

    rviz = ExecuteProcess(
        cmd = [
            'ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_config
        ], 
        output = 'screen'
    )

    return LaunchDescription([
        model_sim, 
        rviz
    ])