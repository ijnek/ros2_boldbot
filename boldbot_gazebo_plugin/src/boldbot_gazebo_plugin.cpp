// Copyright 2022 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "boldbot_gazebo_plugin.hpp"
#include <ignition/plugin/Register.hh>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mx_joint_controller_msgs/msg/joint_command.hpp>

IGNITION_ADD_PLUGIN(
  boldbot_gazebo_plugin::BoldbotGazeboPlugin,
  ignition::gazebo::System,
  boldbot_gazebo_plugin::BoldbotGazeboPlugin::ISystemConfigure,
  boldbot_gazebo_plugin::BoldbotGazeboPlugin::ISystemPreUpdate)

using namespace boldbot_gazebo_plugin;
using JointState = sensor_msgs::msg::JointState;
using JointCommand = mx_joint_controller_msgs::msg::JointCommand;

class boldbot_gazebo_plugin::BoldbotGazeboPluginPrivate
{};

BoldbotGazeboPlugin::BoldbotGazeboPlugin()
: dataPtr(std::make_unique<BoldbotGazeboPluginPrivate>()) {}
void BoldbotGazeboPlugin::Configure(
  const ignition::gazebo::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  ignition::gazebo::EntityComponentManager & _ecm,
  ignition::gazebo::EventManager & _eventMgr)
{}

void BoldbotGazeboPlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  ignition::gazebo::EntityComponentManager & _ecm)
{}
