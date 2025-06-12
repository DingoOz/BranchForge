#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for the behavior tree node'
        ),
        
        Node(
            package='branchforge_generated',
            executable='mybehaviortree_node',
            name='behavior_tree_node',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[
                # Add any parameters here
            ]
        )
    ])
