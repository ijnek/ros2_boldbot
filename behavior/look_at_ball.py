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
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from mx_joint_controller_msgs.msg import JointCommand


class LookAtBall(Node):
    def __init__(self):
        super().__init__("look_at_ball")

        self._joint_states = None

        self.ball_position_sub = self.create_subscription(
            Point, "/ball_position", self.ball_position_callback, 10
        )

        self.joint_states_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )

        self.joint_commands_pub = self.create_publisher(
            JointCommand, "/cm730/joint_commands", 10
        )

    def joint_states_callback(self, joint_states):
        self._joint_states = joint_states

    def ball_position_callback(self, position):
        self._ball_position = position
        
        if self._joint_states is None:
            self.get_logger().warn("No joint states yet")
            return

        off_from_center = (
            position.x - 160
        )  # TODO: should get from camera info/parameters
        if abs(off_from_center) <= 2:
            # Pretty well centered, don't move to prevent jitter
            return

        # positive y in camera is negative radians in pan
        gain = -0.001  # TODO: get from parameter
        delta = gain * off_from_center

        head_pan_idx = self._joint_states.name.index("head-pan")
        head_pan = self._joint_states.position[head_pan_idx]

        target_head_pan = head_pan + delta
        joint_commands = JointCommand()
        joint_commands.name = ["head-pan"]
        joint_commands.position = [target_head_pan]

        self.get_logger().info(
            f"Ball position from center: {off_from_center} - head pan: {head_pan}"
        )

        self.joint_commands_pub.publish(joint_commands)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LookAtBall()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
