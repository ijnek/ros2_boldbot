#include "boldbot_gazebo_plugin/boldbot_gazebo_plugin.hpp"

#include <gazebo/physics/physics.hh>
#include <iostream>

namespace boldbot_gazebo_plugin
{

BoldbotGazeboPlugin::BoldbotGazeboPlugin()
: robot_namespace_{""},
  last_sim_time_{0}
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
      for (int i = 0; i < cmd->name.size(); ++i) {
	joint_targets_[cmd->name[i]] = cmd->position[i];
      }
    }
  );

  // Find joints
  auto allJoints = model_->GetJoints();
  for (auto const & j : allJoints) {
    if (j->GetType() == gazebo::physics::Joint::FIXED_JOINT) {
      continue;
    }

    auto pid = gazebo::common::PID{};
    pid.SetPGain(200.0);
    pid.SetIGain(0.0);
    pid.SetDGain(0.0);

    auto const & name = j->GetName();
    joints_[name] = std::make_pair(j, pid);
    joint_targets_[name] = 0.0;
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
  auto cur_time = world_->SimTime();
  if (last_sim_time_ == 0) {
    last_sim_time_ = cur_time;
    return;
  }

  auto dt = (cur_time - last_sim_time_).Double();
  RCLCPP_DEBUG(ros_node_->get_logger(), std::to_string(dt));

  // Update joint PIDs
  for (auto & j : joints_) {
    auto const & name = j.first;
    auto & joint = j.second.first;
    auto & pid = j.second.second;

    auto error = joint->Position() - joint_targets_[j.first];

    auto force = pid.Update(error, dt);
    joint->SetForce(0, force);
  }

  last_sim_time_ = cur_time;

  rclcpp::spin_some(ros_node_);
}

GZ_REGISTER_MODEL_PLUGIN(BoldbotGazeboPlugin)

}  // namespace boldbot_gazebo_plugin
