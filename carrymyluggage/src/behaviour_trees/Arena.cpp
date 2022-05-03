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

#include "behaviour_trees/Arena.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>

namespace behaviour_trees
{
    
    Arena::Arena(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
    {
    }
    
    
    void 
    Arena::halt()
    {
        ROS_INFO("Arena halt");
    }
    
    
    BT::PortsList 
    Arena::providedPorts() 
    { 
        return { BT::OutputPort<move_base_msgs::MoveBaseGoal>("goal_nav") }; 
    }
    

    BT::NodeStatus
    Arena::tick()
    {
        move_base_msgs::MoveBaseGoal goal;

        ROS_INFO("Envio arena");

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 20;  //coord arbitro
        goal.target_pose.pose.position.y = 20;  //coord arbitro
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;

        setOutput("goal_nav", goal);
        
        return BT::NodeStatus::SUCCESS;
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::Arena>("arena");
}