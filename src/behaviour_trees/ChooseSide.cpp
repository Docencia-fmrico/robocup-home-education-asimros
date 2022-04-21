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

#include "ros/ros.h"

namespace behaviour_trees
{
    ChooseSide::ChooseSide(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config),
	listener(buffer)
    {
    }

    void 
    ChooseSide::halt()
    {
        ROS_INFO("ChooseSide halt");
    }

    BT::NodeStatus
    ChooseSide::tick()
    {
        move_base_msgs::MoveBaseGoal goal;

        if(side_case.find() && buffer.canTransform("map", "person", ros::Time(0), ros::Duration(1.0), &error_))
        {	
			map2person_msg = buffer.lookupTransform("map", "person", ros::Time(0));
      		tf2::fromMsg(map2person_msg, map2person);

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = map2person.getOrigin().x(); // modificarlos: aún más a un lado ¿cuál?
            goal.target_pose.pose.position.y = map2person.getOrigin().y(); // modificarlos: se pare antes
            goal.target_pose.pose.position.z = 0.0;
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 0.0;
            goal.target_pose.pose.orientation.w = 1.0;

            setOutput<move_base_msgs::MoveBaseGoal>("goal_nav", goal);

            // hacer una captura
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
  factory.registerNodeType<behaviour_trees::ChooseSide>("choose_side");
}