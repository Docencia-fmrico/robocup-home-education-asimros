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

#include "behaviour_trees/Localization.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <opencv2/core/types.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

namespace behaviour_trees
{
    
    Localization::Localization(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
    {
        position_ = 0;
        loc_index_ = 0;

        point_[0].x = 2.12; // posición 1 (lo dejemos a un metro)
        point_[0].y = 2.65;
        locs_[0] = 0.0;

        point_[1].x = 0.61;
        point_[1].y = 2.93;
        locs_[1] = 0.0;

        point_[2].x = -0.36;
        point_[2].y = 3.77;
        locs_[2] = 0.0;

        point_[3].x = -0.77;
        point_[3].y = 5.83;
        locs_[3] = 0.0;

        point_[4].x = -0.58;
        point_[4].y = 6.70;
        locs_[4] = 0.0;

        point_[5].x = 1.42;
        point_[5].y = 6.45;
        locs_[5] = 0.0;
    }
    
    
    void 
    Localization::halt()
    {
        ROS_INFO("Localization halt");
    }
    
    
    BT::PortsList 
    Localization::providedPorts() 
    { 
        return { BT::OutputPort<move_base_msgs::MoveBaseGoal>("goal_nav"), BT::OutputPort<int>("count") }; 
    }
    

    BT::NodeStatus
    Localization::tick()
    {
        move_base_msgs::MoveBaseGoal goal;

        ROS_ERROR("Envio posicion %d", position_);

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = point_[position_].x;
        goal.target_pose.pose.position.y = point_[position_].y;  
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = locs_[loc_index_];
        goal.target_pose.pose.orientation.w = 1.0;

        setOutput("goal_nav", goal);
        setOutput("count", position_);
        position_++;
        loc_index_++;

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::Localization>("localization");
}