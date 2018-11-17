#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("dummy_joint_states");

  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states");

  rclcpp::WallRate loop_rate(50);

  auto msg = std::make_shared<sensor_msgs::msg::JointState>();
  msg->name.push_back("head_pan_joint");
  msg->name.push_back("head_tilt_joint");
  msg->name.push_back("l_shoulder_pitch_joint");
  msg->name.push_back("l_shoulder_roll_joint");
  msg->name.push_back("l_elbow_joint");
  msg->name.push_back("r_shoulder_pitch_joint");
  msg->name.push_back("r_shoulder_roll_joint");
  msg->name.push_back("r_elbow_joint");
  msg->name.push_back("l_hip_yaw_joint");
  msg->name.push_back("l_hip_roll_joint");
  msg->name.push_back("l_hip_pitch_joint");
  msg->name.push_back("l_knee_joint");
  msg->name.push_back("l_ankle_pitch_joint");
  msg->name.push_back("l_ankle_roll_joint");
  for (size_t i = 0; i < msg->name.size(); ++i)
    msg->position.push_back(0.0);

  auto counter = 0.0;
  auto joint_value = 0.0;
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  while (rclcpp::ok()) {
    counter += 0.1;
    joint_value = std::sin(counter);

    for (size_t i = 0; i < msg->name.size(); ++i) {
      msg->position[i] = joint_value;
    }

    msg->header.stamp = ros_clock.now();

    joint_state_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
