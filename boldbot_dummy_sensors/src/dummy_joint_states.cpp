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
  msg->name.push_back("shoulder-pitch-r");
  msg->name.push_back("shoulder-pitch-l");
  msg->name.push_back("shoulder-roll-r");	
  msg->name.push_back("shoulder-roll-l");
  msg->name.push_back("elbow-joint-r");	
  msg->name.push_back("elbow-joint-l");
  msg->name.push_back("hip-yaw-r");
  msg->name.push_back("hip-yaw-l");
  msg->name.push_back("hip-roll-r");	
  msg->name.push_back("hip-roll-l");
  msg->name.push_back("hip-pitch-r");	
  msg->name.push_back("hip-pitch-l");
  msg->name.push_back("knee-r");  
  msg->name.push_back("knee-l");
  msg->name.push_back("ankle-pitch-r");  
  msg->name.push_back("ankle-pitch-l");
  msg->name.push_back("ankle-roll-r");  
  msg->name.push_back("ankle-roll-l");
  msg->name.push_back("head-pan");
  msg->name.push_back("head-tilt");
  
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
