import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Standalone precision-landing requester. Run alongside a separately-launched
    # offboard_master (with fence.* params consistent with the landing altitudes)
    # and the aruco_tracker vision node publishing /target_pose.
    config = os.path.join(
        get_package_share_directory('aruco_land'), 'config', 'aruco_land.yaml')

    return LaunchDescription([
        Node(
            package='aruco_land',
            executable='aruco_land_node',
            name='aruco_land_node',
            output='screen',
            parameters=[config],
        ),
    ])
