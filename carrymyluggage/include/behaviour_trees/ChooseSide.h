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

#ifndef BEHAVIOUR_TREES_CHOOSESIDE_H
#define BEHAVIOUR_TREES_CHOOSESIDE_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"

#include <string>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>


#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

#include "detect_case/SideCase.h"

namespace behaviour_trees
{

class ChooseSide : public BT::ActionNodeBase
{
    public:
        explicit ChooseSide(const std::string& name, const BT::NodeConfiguration& config);
        
        void halt();

        static BT::PortsList providedPorts() 
    	{ 
        	return { BT::OutputPort<move_base_msgs::MoveBaseGoal>("goal_nav")}; 
    	}

        BT::NodeStatus tick();

    private:
        ros::NodeHandle nh_;
        detect_case::SideCase case_;
        std::string error_;
		tf2_ros::Buffer buffer;
        tf2_ros::TransformListener listener;
		geometry_msgs::TransformStamped map2odom_msg;
    	tf2::Stamped<tf2::Transform> map2odom;
		geometry_msgs::TransformStamped odom2bf_msg;
    	tf2::Stamped<tf2::Transform> odom2bf;
	 	geometry_msgs::TransformStamped bf2person_msg;
    	tf2::Stamped<tf2::Transform> bf2person;

		tf2::Transform map2person;
};

}  // namespace behaviour_trees

#endif  // BEHAVIOUR_TREES_CHOOSESIDE_H