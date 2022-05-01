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

#ifndef BEHAVIOUR_TREES_FOLLOWPOINT_H
#define BEHAVIOUR_TREES_FOLLOWPOINT_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <string>
#include <move_base_msgs/MoveBaseAction.h>
#include "behaviour_trees/BTNavAction.h"

#include "ros/ros.h"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

namespace behaviour_trees
{

class FollowPoint : public BTNavAction
{
  public:
    explicit FollowPoint(const std::string& name,
    const std::string & action_name,
    const BT::NodeConfiguration & config);

    void on_halt() override;
    BT::NodeStatus on_tick() override;
    void on_start() override;
    void on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) override;

	static BT::PortsList providedPorts() 
	{ 
    	return { BT::InputPort<move_base_msgs::MoveBaseGoal>("goal_nav")}; 
	}

  private:
    int counter_;
    bool isDifferent(move_base_msgs::MoveBaseGoal newgoal);
	  move_base_msgs::MoveBaseGoal goal_;
};

}  // namespace behaviour_trees

#endif  // BEHAVIOUR_TREES_FOLLOWPOINT_H