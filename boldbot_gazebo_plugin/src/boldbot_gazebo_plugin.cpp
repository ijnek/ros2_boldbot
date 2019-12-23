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
  model_ = model;
  world_ = model_->GetWorld();
  auto physicsEngine = world_->Physics();
  physicsEngine->SetParam("friction_model", std::string{"cone_model"});

  if (sdf->HasElement("robotNamespace")) {
    robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  ros_node_ = rclcpp::Node::make_shared("bolbot_gazebo");
  RCLCPP_INFO(ros_node_->get_logger(), "Loading Boldbot Gazebo Plugin");
  joint_command_sub_ = ros_node_->create_subscription<JointCommand>(
    "/cm730/joint_commands",
    10,
    [ = ](JointCommand::SharedPtr cmd) {
      RCLCPP_INFO(ros_node_->get_logger(), "Received joint commands!");
    }
  );

  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&BoldbotGazeboPlugin::Update, this));
}

void BoldbotGazeboPlugin::Update()
{
  rclcpp::spin_some(ros_node_);
}

GZ_REGISTER_MODEL_PLUGIN(BoldbotGazeboPlugin)

}  // namespace boldbot_gazebo_plugin
