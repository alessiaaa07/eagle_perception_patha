import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('cone_detection'),
        'config',
        'cone_detection.yaml',
    )

    cone_detection_node = Node(
        package='cone_detection',
        executable='cone_detection_node',
        name='cone_detection_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([cone_detection_node])