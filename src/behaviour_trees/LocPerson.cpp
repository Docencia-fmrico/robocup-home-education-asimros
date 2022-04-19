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

#include "behaviour_trees/LocPerson.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

namespace behaviour_trees
{
    
    LocPerson::LocPerson(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
    {
    }
    
    
    void 
    LocPerson::halt()
    {
        ROS_INFO("LocPerson halt");
    }
    
    
    BT::PortsList 
    LocPerson::providedPorts() 
    { 
        return { BT::OutputPort<move_base_msgs::MoveBaseGoal>("goal_nav") }; 
    }
    

    BT::NodeStatus
    LocPerson::tick()
    {
        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener listener(buffer);
        move_base_msgs::MoveBaseGoal goal;

        if(buffer.canTransform("base_footprint", "person/0", ros::Time(0), ros::Duration(1.0), &error_))
        {
            ROS_INFO("I have seen a person");

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = bf2person_.getOrigin().x();
            goal.target_pose.pose.position.y = bf2person_.getOrigin().y();
            goal.target_pose.pose.position.z = 0.0;
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 0.0;
            goal.target_pose.pose.orientation.w = 1.0;

            setOutput("goal_nav", goal);

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_INFO("I haven't seen a person yet");
            return BT::NodeStatus::FAILURE;  
        }
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::LocPerson>("loc_person");
}