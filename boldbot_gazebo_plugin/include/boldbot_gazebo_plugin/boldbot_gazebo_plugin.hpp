// Copyright 2019 Bold Hearts
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

#include <gazebo/common/Plugin.hh>

namespace boldbot_gazebo_plugin
{
  
class BoldbotGazeboPlugin : public gazebo::ModelPlugin
{
public:
  BoldbotGazeboPlugin();
  
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
  
private:
  std::string robot_namespace_;

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
};

}

#endif  // BOLDBOT_GAZEBO_PLUGIN__BOLDBOT_GAZEBO_PLUGIN_HPP_
