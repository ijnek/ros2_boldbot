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

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('walking_straight')
    publisher = node.create_publisher(Twist, '/walking/command', 10)

    linear = Vector3()
    linear.x = 0.1
    linear.y = 0.0
    linear.z = 0.0
    angular = Vector3()
    angular.x = 0.0
    angular.y = 0.0
    angular.z = 0.0

    msg = Twist(
        linear=linear,
        angular=angular
    )

    def timer_callback():
        node.get_logger().info('Publishing: {}'.format(msg))
        publisher.publish(msg)

    timer = node.create_timer(1, timer_callback)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
