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

#include "behaviour_trees/ChooseSide.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

#include "nav_msgs/Path.h"
#include "nav_msgs/GetPlan.h"
#include "geometry_msgs/PoseStamped.h"

#include "ros/ros.h"

namespace behaviour_trees
{

    ChooseSide::ChooseSide(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config),
	listener(buffer)
    {
		client = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    }

    void 
    ChooseSide::halt()
    {
        ROS_INFO("ChooseSide halt");
    }

    BT::NodeStatus
    ChooseSide::tick()
    {
        ROS_ERROR("Porfi apunta");

        ROS_ERROR("Detectando lado");

        ROS_ERROR("Me giro a un lado");
		    return BT::NodeStatus::SUCCESS;
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::ChooseSide>("choose_side");
}