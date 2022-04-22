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
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mx_joint_controller_msgs/msg/joint_command.hpp>
#include <sdf/sdf.hh>

IGNITION_ADD_PLUGIN(
  boldbot_gazebo_plugin::BoldbotGazeboPlugin,
  ignition::gazebo::System,
  boldbot_gazebo_plugin::BoldbotGazeboPlugin::ISystemConfigure,
  boldbot_gazebo_plugin::BoldbotGazeboPlugin::ISystemPreUpdate)

using namespace boldbot_gazebo_plugin;
using namespace std::chrono_literals;
using JointState = sensor_msgs::msg::JointState;
using JointCommand = mx_joint_controller_msgs::msg::JointCommand;

class boldbot_gazebo_plugin::BoldbotGazeboPluginPrivate
{
public:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<JointCommand>::SharedPtr joint_command_sub_;

  std::map<std::string, double> joint_targets_;

  std::map<std::string, std::pair<ignition::gazebo::Entity, ignition::math::PID>> joints_;

  ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};

  std::chrono::duration<double> last_update_time_;
  std::chrono::duration<double> update_period_ = 8ms;

  std::string robot_namespace_;

  bool firstUpdate = true;
};

BoldbotGazeboPlugin::BoldbotGazeboPlugin()
: dataPtr(std::make_unique<BoldbotGazeboPluginPrivate>()) {}
void BoldbotGazeboPlugin::Configure(
  const ignition::gazebo::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  ignition::gazebo::EntityComponentManager & _ecm,
  ignition::gazebo::EventManager &)
{
  rclcpp::init(0, nullptr);
  dataPtr->ros_node_ = rclcpp::Node::make_shared("boldbot_gazebo_plugin");
  dataPtr->joint_state_pub_ = dataPtr->ros_node_->create_publisher<JointState>(
    "/joint_states", 10);
  dataPtr->joint_command_sub_ = dataPtr->ros_node_->create_subscription<JointCommand>(
    "/cm730/joint_commands",
    10,
    [ = ](JointCommand::SharedPtr cmd) {
      for (size_t i = 0; i < cmd->name.size(); ++i) {
        dataPtr->joint_targets_[cmd->name[i]] = cmd->position[i];
      }
    }
  );

  if (_sdf->HasElement("robotNamespace")) {
    dataPtr->robot_namespace_ = _sdf->FindElement("robotNamespace")->GetValue()->GetAsString() +
      "/";
  }

  // Get model and world references
  dataPtr->model_ = ignition::gazebo::Model(_entity);

  // Find joints
  auto allJoints = dataPtr->model_.Joints(_ecm);
  for (auto const & j : allJoints) {
    auto jointTypeComp = _ecm.Component<ignition::gazebo::components::JointType>(j);
    auto nameComp = _ecm.Component<ignition::gazebo::components::Name>(j);
    if (nameComp && jointTypeComp) {
      sdf::JointType jointType = jointTypeComp->Data();
      if (jointType == sdf::JointType::FIXED) {
        continue;
      }

      auto pid = ignition::math::PID{};
      pid.SetPGain(200.0);
      pid.SetIGain(0.0);
      pid.SetDGain(0.0);

      auto const & name = nameComp->Data();
      dataPtr->joints_[name] = std::make_pair(j, pid);
      dataPtr->joint_targets_[name] = 0.0;

      auto jointPosComp =
        _ecm.Component<ignition::gazebo::components::JointPosition>(j);
      if (!jointPosComp) {
        _ecm.CreateComponent(
          j, ignition::gazebo::components::JointPosition());
      }
    }
  }

  RCLCPP_DEBUG(dataPtr->ros_node_->get_logger(), "Got joints:");
  for (auto const & j : dataPtr->joints_) {
    RCLCPP_DEBUG(dataPtr->ros_node_->get_logger(), "%s", j.first.c_str());
  }
}

void BoldbotGazeboPlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  ignition::gazebo::EntityComponentManager & _ecm)
{
  rclcpp::spin_some(dataPtr->ros_node_);

  auto cur_time = _info.simTime;
  if (dataPtr->firstUpdate) {
    dataPtr->last_update_time_ = cur_time;
    dataPtr->firstUpdate = false;
    return;
  }

  // Publish joint states
  auto update_dt = cur_time - dataPtr->last_update_time_;
  if (update_dt >= dataPtr->update_period_) {
    auto msg = JointState{};
    msg.header.stamp = dataPtr->ros_node_->now();

    for (auto & j : dataPtr->joints_) {
      auto const & name = j.first;
      auto & joint = j.second.first;

      auto jointPosComp =
        _ecm.Component<ignition::gazebo::components::JointPosition>(joint);
      if (jointPosComp) {
        auto position = jointPosComp->Data().at(0);
        msg.name.push_back(name);
        msg.position.push_back(position);
      } else {
        std::cout << "Couldn't find joint pos!" << std::endl;
      }
    }
    dataPtr->joint_state_pub_->publish(msg);
    dataPtr->last_update_time_ = cur_time;
  }

  // Update joint PIDs
  for (auto & j : dataPtr->joints_) {
    auto & joint = j.second.first;
    auto & pid = j.second.second;

    auto jointPosComp =
      _ecm.Component<ignition::gazebo::components::JointPosition>(joint);
    if (jointPosComp) {
      auto position = jointPosComp->Data().at(0);
      auto error = position - dataPtr->joint_targets_[j.first];
      auto force = pid.Update(error, _info.dt);

      auto forceComp =
        _ecm.Component<ignition::gazebo::components::JointForceCmd>(joint);
      if (forceComp == nullptr) {
        _ecm.CreateComponent(joint, ignition::gazebo::components::JointForceCmd({force}));
      } else {
        forceComp->Data()[0] = force;
      }
    }
  }
}
