from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    optical_flow_node = Node(
        package='optical_flow',
        executable='optical_flow_node',
        name='optical_flow_node',
        output='screen'
    )

    return LaunchDescription([
        optical_flow_node
    ])
