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
from scripts import GazeboRosPaths


def generate_launch_description():
    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    env = {
        'GAZEBO_MODEL_PATH': model_path,
        'GAZEBO_PLUGIN_PATH': plugin_path,
        'GAZEBO_RESOURCE_PATH': media_path,
    }

    urdf_prefix = get_package_share_directory('boldbot_description')
    urdf_file = os.path.join(urdf_prefix, 'urdf', 'boldbot.urdf')

    world_prefix = get_package_share_directory('boldbot_sim')
    world_file = os.path.join(world_prefix, 'worlds', 'rchl_kid_2019.world')

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    'gazebo',
                    '-s',
                    'libgazebo_ros_init.so',
                    '-s',
                    'libgazebo_ros_factory.so',
                    world_file,
                ],
                output='screen',
                additional_env=env,
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity',
                    'boldbot',
                    '-x',
                    '-1',
                    '-y',
                    '0',
                    '-z',
                    '.41',
                    '-b',
                    '-file',
                    urdf_file,
                ],
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                arguments=[urdf_file],
            ),
        ]
    )
