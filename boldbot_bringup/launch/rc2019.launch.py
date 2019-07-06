import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown


def generate_launch_description():
    urdf = os.path.join(
        get_package_share_directory("boldbot_description"), "urdf", "boldbot.urdf"
    )
    params = os.path.join(
        get_package_share_directory("boldbot_bringup"), "config", "rc2019_params.yml"
    )
    return LaunchDescription(
        [
            # CM730
            Node(
                package="cm730driver",
                node_executable="cm730driver_node",
                output="screen",
                on_exit=Shutdown(),
            ),
            Node(
                package="cm730controller",
                node_executable="cm730controller_node",
                output="screen",
                on_exit=Shutdown(),
            ),
            Node(
                package="mx_joint_controller",
                node_executable="mx_joint_controller_node",
                output="screen",
                on_exit=Shutdown(),
            ),
            Node(
                package="imu_publisher",
                node_executable="imu_publisher_node",
                output="screen",
                on_exit=Shutdown(),
            ),
            Node(
                package="button_publisher",
                node_executable="button_publisher_node",
                output="screen",
                on_exit=Shutdown(),
            ),
            # State
            Node(
                package="robot_state_publisher",
                node_executable="robot_state_publisher",
                output="screen",
                arguments=[urdf],
            ),
            # Vision
            Node(
                package="v4l2_camera",
                node_executable="v4l2_camera_node",
                parameters=[params],
                output="screen",
                on_exit=Shutdown(),
            ),
            Node(
                package="tflite",
                node_executable="tflite_node",
                output="screen",
                on_exit=Shutdown(),
            ),
            Node(
                package="bold_vision",
                node_executable="bold_vision_node",
                output="screen",
                on_exit=Shutdown(),
            ),
            # Behavior
            Node(
                package="behavior",
                node_executable="look_at_ball",
                output="screen",
                on_exit=Shutdown(),
            ),
            # Motion scripts
            Node(
                package="motion_script",
                node_executable="motion_script",
                output="screen",
                on_exit=Shutdown(),
            ),
            # Game controller
            Node(
                package="game_controller",
                node_executable="game_controller_node",
                output="screen"
            )
        ]
    )
