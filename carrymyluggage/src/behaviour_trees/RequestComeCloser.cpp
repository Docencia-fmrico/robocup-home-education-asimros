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

#include "behaviour_trees/RequestComeCloser.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>

#include "sound/Speaker.h"
#include "ros/ros.h"

namespace behaviour_trees
{
    RequestComeCloser::RequestComeCloser(const std::string& name)
    : BT::ActionNodeBase(name, {})
    {
    }

    void 
    RequestComeCloser::halt()
    {
        ROS_INFO("RequestComeCloser halt");
    }

    BT::NodeStatus
    RequestComeCloser::tick()
    {
        ROS_ERROR("Could you come closer?");
        speaker_.speak("Could you come closer?");
        return BT::NodeStatus::SUCCESS;
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::RequestComeCloser>("request_come_closer");
}