// Copyright 2019 Intelligent Robotics Lab
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

#include "behaviour_trees/Begin.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>
#include "sound/Listener.h"

#include "ros/ros.h"

namespace behaviour_trees
{
    Begin::Begin(const std::string& name)
    : BT::ActionNodeBase(name, {})
    {
      first_ = true;
    }

    void
    Begin::halt()
    {
      ROS_INFO("Begin halt");
    }

    BT::NodeStatus
    Begin::tick()
    {
      if(first_){
        first_ = false;
        listen_ts_ = ros::Time::now();
      }
      if((ros::Time::now() - listen_ts_).toSec() < LISTEN_TIME){
        listener_.listen();

        if(!listener_.answer().compare("false"))
        {
          ROS_ERROR("Me ha dicho de empezar");
          return BT::NodeStatus::SUCCESS;
        }
      }
      else{
        return BT::NodeStatus::SUCCESS;
      }
      

      ROS_ERROR("Aún no empiezo");
      return BT::NodeStatus::RUNNING;
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::Begin>("begin");
}