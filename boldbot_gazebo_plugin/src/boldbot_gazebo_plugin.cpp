#include "boldbot_gazebo_plugin/boldbot_gazebo_plugin.hpp"

#include <gazebo/physics/physics.hh>
#include <iostream>

namespace boldbot_gazebo_plugin
{

BoldbotGazeboPlugin::BoldbotGazeboPlugin()
: robot_namespace_{""}
{
}

void BoldbotGazeboPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Get model and world references
  model_ = model;
  world_ = model_->GetWorld();
  auto physicsEngine = world_->Physics();
  physicsEngine->SetParam("friction_model", std::string{"cone_model"});

  if (sdf->HasElement("robotNamespace")) {
    robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  // Set up ROS node and subscribers and publishers
  ros_node_ = rclcpp::Node::make_shared("bolbot_gazebo");
  RCLCPP_INFO(ros_node_->get_logger(), "Loading Boldbot Gazebo Plugin");
  joint_command_sub_ = ros_node_->create_subscription<JointCommand>(
    "/cm730/joint_commands",
    10,
    [ = ](JointCommand::SharedPtr cmd) {
      RCLCPP_INFO(ros_node_->get_logger(), "Received joint commands!");
    }
  );

  // Find joints
  auto allJoints = model_->GetJoints();
  for (auto const & j : allJoints) {
    if (j->GetType() == gazebo::physics::Joint::FIXED_JOINT) {
      continue;
    }

    joints_[j->GetName()] = j;
  }

  RCLCPP_DEBUG(ros_node_->get_logger(), "Got joints:");
  for (auto const & j : joints_) {
    RCLCPP_DEBUG(ros_node_->get_logger(), j.first);
  }

  // Hook into simulation update loop
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&BoldbotGazeboPlugin::Update, this));
}

void BoldbotGazeboPlugin::Update()
{
  rclcpp::spin_some(ros_node_);
}

GZ_REGISTER_MODEL_PLUGIN(BoldbotGazeboPlugin)

}  // namespace boldbot_gazebo_plugin
