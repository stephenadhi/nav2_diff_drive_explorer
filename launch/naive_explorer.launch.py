from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # get parameter file path
    params_file_path = os.path.join(
        get_package_share_directory('nav2_diff_drive_explorer'),
        'params',
        'naive_explorer.yaml'
    )
    # laod parameters
    with open(params_file_path, 'r') as file:
        naive_explorer_params = yaml.safe_load(file)['naive_explorer']['ros_parameters']
    
    naive_explorer_cmd = Node(
        package='nav2_diff_drive_explorer',
        executable='naive_explorer',
        name='naive_explorer_node',
        output='screen',
        parameters=[naive_explorer_params],
    )

    ld = LaunchDescription()

    ld.add_action(naive_explorer_cmd)

    return ld