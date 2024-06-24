import os
from launch import LaunchDescription
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_potential_function',
            executable='simulation',
            name='navigation_potential_function_node_0',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('navigation_potential_function'),
                'config', 'params.yaml')]
        ),
        Node(
            package='navigation_potential_function',
            executable='simulation',
            name='navigation_potential_function_node_1',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('navigation_potential_function'),
                'config', 'params.yaml')]
        ),
        Node(
            package='navigation_potential_function',
            executable='simulation',
            name='navigation_potential_function_node_2',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('navigation_potential_function'),
                'config', 'params.yaml')]
        )
    ])