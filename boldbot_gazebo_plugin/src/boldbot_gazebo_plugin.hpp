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

#ifndef BOLDBOT_GAZEBO_PLUGIN__BOLDBOT_GAZEBO_PLUGIN_HPP_
#define BOLDBOT_GAZEBO_PLUGIN__BOLDBOT_GAZEBO_PLUGIN_HPP_

#include <ignition/gazebo/System.hh>

namespace boldbot_gazebo_plugin
{

class BoldbotGazeboPluginPrivate;

class BoldbotGazeboPlugin
  : public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
public:
  explicit BoldbotGazeboPlugin();
  ~BoldbotGazeboPlugin() override = default;
  void Configure(
    const ignition::gazebo::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    ignition::gazebo::EntityComponentManager & _ecm,
    ignition::gazebo::EventManager & _eventMgr) override;
  void PreUpdate(
    const ignition::gazebo::UpdateInfo & _info,
    ignition::gazebo::EntityComponentManager & _ecm) override;

private:
    private: std::unique_ptr<BoldbotGazeboPluginPrivate> dataPtr;
};

}  // namespace boldbot_gazebo_plugin

#endif  // BOLDBOT_GAZEBO_PLUGIN__BOLDBOT_GAZEBO_PLUGIN_HPP_
