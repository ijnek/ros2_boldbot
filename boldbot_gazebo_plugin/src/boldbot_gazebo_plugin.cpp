#include "boldbot_gazebo_plugin/boldbot_gazebo_plugin.hpp"

#include <gazebo/physics/physics.hh>
#include <iostream>

namespace boldbot_gazebo_plugin {

BoldbotGazeboPlugin::BoldbotGazeboPlugin()
: robot_namespace_{""}
{
  std::cout << "Creating Boldbot Gazebo Plugin\n";
}

void BoldbotGazeboPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  std::cout << "Loading Boldbot Gazebo Plugin\n";
  model_ = model;
  world_ = model_->GetWorld();
  auto physicsEngine = world_->Physics();
  physicsEngine->SetParam("friction_model", std::string{"cone_model"});
  
  if (sdf->HasElement("robotNamespace"))
    robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  
}

GZ_REGISTER_MODEL_PLUGIN(BoldbotGazeboPlugin)

}  // namespace boldbot_gazebo_plugin
