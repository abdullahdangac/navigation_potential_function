import os
from launch import LaunchDescription
#from launch_ros.actions import SetParameter
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen',
        ),
        TimerAction(
            period=0.1,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/kill', 'turtlesim/srv/Kill', '{name: "turtle1"}'],
                    output='screen'
                ),
                TimerAction(
                period=1.0,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', '{x: 1.0, y: 5.0, theta: 0.0, name: "turtle0"}'],
                        output='screen'
                    ),
                    ExecuteProcess(
                        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', '{x: 3.0, y: 2.0, theta: -1.0, name: "turtle1"}'],
                        output='screen'
                    ),
                    ExecuteProcess(
                        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', '{x: 7.0, y: 8.0, theta: 1.0, name: "turtle2"}'],
                        output='screen'
                    ),
                    ExecuteProcess(
                        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', '{x: 8.0, y: 3.0, theta: 1.0, name: "turtle3"}'],
                        output='screen'
                    )]
                )
            ]
        )
    ])