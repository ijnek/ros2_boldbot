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

import rclpy
from rclpy.node import Node

# for linear acceleration
from geometry_msgs.msg import Vector3

# for orientation
from geometry_msgs.msg import Quaternion

from sensor_msgs.msg import Imu

class GetUp(Node):

    def __init__(self):
        super().__init__('get_up')
        self.sub = self.create_subscription(Imu, '/imu/data', self.get_up_callback, 10)

    def get_up_callback(self, msg):
	if isinstance(msg.linear_acceleration, Vector3):
		self.get_logger().info("({},{},{})".format(fmt(msg.linear_acceleration.x), fmt(msg.linear_acceleration.y), fmt(msg.linear_acceleration.z)))

def main(args=None):
    rclpy.init(args=args)

    node = GetUp()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
