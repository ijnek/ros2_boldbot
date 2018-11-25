import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('boldbot_description'),
                        'urdf', 'boldbot.urdf')
    return LaunchDescription([
        Node(package='robot_state_publisher', node_executable='robot_state_publisher',
             output='screen', arguments=[urdf]),
        Node(package='cm730', node_executable='cm730_state',
             output=None),
        Node(package='usb_cam', node_executable='usb_cam_node',
             output=None)

])
