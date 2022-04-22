# Copyright 2019 Bold Hearts
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    urdf_prefix = get_package_share_directory('boldbot_description')
    urdf_file = os.path.join(urdf_prefix, 'urdf', 'boldbot.urdf')

    world_prefix = get_package_share_directory('boldbot_sim')
    world_file = os.path.join(world_prefix, 'worlds', 'rchl_kid_2019.world')

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'launch',
                    'ros_ign_gazebo',
                    'ign_gazebo.launch.py',
                    'ign_args:='+world_file
                ],
                output='screen',
            ),
            Node(package='ros_ign_gazebo', executable='create',
                arguments=[
                    '-name', 'boldbot',
                    '-topic', 'robot_description',
                    '-x', '1',
                    '-y', '0',
                    '-z', '.41',
                    ],
                output='screen',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                arguments=[urdf_file],
            ),
            Node(
                package='ros_ign_bridge',
                executable='parameter_bridge',
                arguments=[
                        # Clock (IGN -> ROS2)
                        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                        # Joint states (IGN -> ROS2)
                        '/world/default/model/boldbot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                        # Image (IGN -> ROS2)
                        '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
                        # Imu (IGN -> ROS2)
                        '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                        ],
                remappings=[
                    ('/world/default/model/boldbot/joint_state', 'joint_states'),
                ],
                output='screen'
            )
        ]
    )
