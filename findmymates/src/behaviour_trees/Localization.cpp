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
    : BT::ActionNodeBase(name, config),
      listener(buffer)
    {
        client = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
        // Coordenadas inicio de la nav, después se actualizan con siguiente destino (es donde está el árbitro)
        start.header.frame_id = "map";
		start.pose.position.x = 0.0;  
        start.pose.position.y = 0.0;  
        start.pose.position.z = 0.0;
        start.pose.orientation.x = 0.0;
        start.pose.orientation.y = 0.0;
        start.pose.orientation.z = 0.0;
        start.pose.orientation.w = 1.0;
        position_ = 0;
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
        nav_msgs::GetPlan srv;
        srv.request.goal.header.frame_id = "map";
        srv.request.goal.pose.position.x = point_[position_].x;  //coord arbitro
        srv.request.goal.pose.position.y = point_[position_].y; //coord arbitro
        srv.request.goal.pose.position.z = 0.0;
        srv.request.goal.pose.orientation.x = 0.0;
        srv.request.goal.pose.orientation.y = 0.0;
        srv.request.goal.pose.orientation.z = 0.0;
        srv.request.goal.pose.orientation.w = 1.0;
        srv.request.tolerance = 1.0;

        if (client.call(srv))
        {
            auto index = Localization::calc_index(srv.response.plan.poses) - 2;
            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose = srv.response.plan.poses[index];
            start = goal.target_pose;

            setOutput<move_base_msgs::MoveBaseGoal>("goal_nav", goal);
            setOutput<int>("count", position_);
            position_++;
            ROS_ERROR("x = %f y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("Failed to call service distance");
            return BT::NodeStatus::RUNNING; 
        }
        
        
    
		ROS_ERROR("Unable to transform");
        return BT::NodeStatus::FAILURE;
    }

    unsigned long Localization::calc_index(auto & poses)
    {
        int size;
        double x;
        double y;
        double diffx;
        double diffy;
        double dist;
        unsigned long index;
        int i;

        size = poses.size();

        for (i = 0; i < size; i++)
        {
            x = poses[i].pose.position.x;
            y = poses[i].pose.position.y;

            diffx = abs(point_[position_].x - x);
            diffy = abs(point_[position_].y - y);

            dist = sqrt(diffx * diffx + diffy * diffy);
            if (dist <= 1.0) return (unsigned long)i;
        }

        return -1;
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::Localization>("localization");
}