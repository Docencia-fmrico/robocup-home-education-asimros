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

#include "ros/ros.h"

namespace behaviour_trees
{
    ChooseSide::ChooseSide(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
    {
    }

    void 
    ChooseSide::halt()
    {
        ROS_INFO("ChooseSide halt");
    }

    BT::PortsList 
    ChooseSide::providedPorts() 
    { 
        return { BT::OutputPort<move_base_msgs::MoveBaseGoal>("goal_nav") }; 
    }

    BT::NodeStatus
    ChooseSide::tick()
    {
        move_base_msgs::MoveBaseGoal goal;

        if(!side_case.find())
        {
            return BT::NodeStatus::RUNNING;  
        }
        else
        {
            // los timestapms no se comprueban -> sabemos que hay persona por side_case ????
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = bf2person_.getOrigin().x(); // modificarlos: aún más a un lado ¿cuál?
            goal.target_pose.pose.position.y = bf2person_.getOrigin().y(); // modificarlos: se pare antes
            goal.target_pose.pose.position.z = 0.0;
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 0.0;
            goal.target_pose.pose.orientation.w = 1.0;

            setOutput("goal_nav", goal);

            // hacer una captura
            return BT::NodeStatus::SUCCESS;
        }
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::ChooseSide>("choose_side");
}