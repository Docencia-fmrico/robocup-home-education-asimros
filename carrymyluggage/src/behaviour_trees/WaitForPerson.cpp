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

#include "behaviour_trees/WaitForPerson.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>
#include "ros/ros.h"

namespace behaviour_trees
{
    WaitForPerson::WaitForPerson(const std::string& name)
    : BT::ActionNodeBase(name, {}),
   	  listener(buffer)
    {
        first_ = true;
        activation_ = nh_.advertise<std_msgs::Int64>("/activation", 1);
    }

    void 
    WaitForPerson::halt()
    {
        ROS_INFO("WaitForPerson halt");
    }

    BT::NodeStatus
    WaitForPerson::tick()
    {
        if (first_) {
            msg_.data = 1;
            activation_.publish(msg_);
            first_ = false;
        }
        if(buffer.canTransform("map", "person", ros::Time(0), ros::Duration(1.0), &error_))
        {
            ROS_ERROR("I have seen a person");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("I haven't seen a person yet");
            return BT::NodeStatus::RUNNING;  
        }
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::WaitForPerson>("wait_for_person");
}