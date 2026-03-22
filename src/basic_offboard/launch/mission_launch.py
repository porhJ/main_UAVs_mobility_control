from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'desired_laps',
            default_value='3',
            description='Number of endurance laps before switching to mapping mission',
        ),

        Node(
            package='basic_offboard',
            executable='offboard_master',
            name='offboard_master',
            output='screen',
        ),

        Node(
            package='basic_offboard',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[{
                'desired_laps': LaunchConfiguration('desired_laps'),
            }],
        ),
    ])
