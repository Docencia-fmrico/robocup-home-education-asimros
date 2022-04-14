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

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

namespace behaviour_trees
{
    WaitForPerson::WaitForPerson(const std::string& name)
    : BT::ActionNodeBase(name, {})
    {
    }

    void 
    WaitForPerson::halt()
    {
        ROS_INFO("WaitForPerson halt");
    }

    BT::NodeStatus
    WaitForPerson::tick()
    {
        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener listener(buffer);

        if(buffer.canTransform("base_footprint", "person/0", ros::Time(0), ros::Duration(1.0), &error))
        {
            ROS_INFO("I have seen a person");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::RUNNING;  
        }
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::WaitForPerson>("WaitForPerson");
}