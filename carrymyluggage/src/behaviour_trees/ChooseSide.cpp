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
        if(buffer.canTransform("map", "person", ros::Time(0), ros::Duration(1.0), &error_))
        {			
			map2person_msg = buffer.lookupTransform("map", "person", ros::Time(0));
      		tf2::fromMsg(map2person_msg, map2person);

			nav_msgs::GetPlan srv;
			srv.request.goal.header.frame_id = "map";
        	srv.request.goal.pose.position.x = map2person.getOrigin().x();  //coord arbitro
        	srv.request.goal.pose.position.y = map2person.getOrigin().y();  //coord arbitro
        	srv.request.goal.pose.position.z = 0.0;
        	srv.request.goal.pose.orientation.x = 0.0;
        	srv.request.goal.pose.orientation.y = 0.0;
        	srv.request.goal.pose.orientation.z = 0.0;
        	srv.request.goal.pose.orientation.w = 1.0;

			srv.request.start.header.frame_id = "map";
			srv.request.start.pose.position.x = 3.0;  //coord arbitro
        	srv.request.start.pose.position.y = 2.0;  //coord arbitro
        	srv.request.start.pose.position.z = 0.0;
        	srv.request.start.pose.orientation.x = 0.0;
        	srv.request.start.pose.orientation.y = 0.0;
        	srv.request.start.pose.orientation.z = 0.0;
        	srv.request.start.pose.orientation.w = 1.0;

			srv.request.tolerance = 1.0;

			if (client.call(srv))
  			{
    			auto index = srv.response.plan.poses.size() - 8;
				move_base_msgs::MoveBaseGoal goal;

        		goal.target_pose = srv.response.plan.poses[index];

        		setOutput<move_base_msgs::MoveBaseGoal>("goal_nav", goal);
				return BT::NodeStatus::SUCCESS;
  			}
  			else
  			{
    			ROS_ERROR("Failed to call service distance");
				return BT::NodeStatus::RUNNING;
  			}
			
        	
		}
		ROS_ERROR("Unable to transform");
        return BT::NodeStatus::RUNNING; 
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::ChooseSide>("choose_side");
}