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

#include "behaviour_trees/FollowPoint.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>

#include "ros/ros.h"

namespace behaviour_trees
{

FollowPoint::FollowPoint(
  const std::string& name,
  const std::string & action_name,
  const BT::NodeConfiguration & config)
: BTNavAction(name, action_name, config), counter_(0)
{
}

void
FollowPoint::on_halt()
{
  ROS_INFO("Move halt");
}

void
FollowPoint::on_start()
{
  move_base_msgs::MoveBaseGoal goal = getInput<move_base_msgs::MoveBaseGoal>("goal_nav").value();
  set_goal(goal);

  ROS_INFO("Move start");
}

BT::NodeStatus
FollowPoint::on_tick()
{
  ROS_INFO("Move tick");
  
  if (counter_++ == 20)
  {
    std::cerr << "New Goal===========================" << std::endl;

    move_base_msgs::MoveBaseGoal goal = getInput<move_base_msgs::MoveBaseGoal>("goal_nav").value();
	set_goal(goal);
  }

  return BT::NodeStatus::RUNNING;
}

void
FollowPoint::on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
}

}  // namespace behavior_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<behaviour_trees::FollowPoint>(
        name, "move_base", config);
    };

  factory.registerBuilder<behaviour_trees::FollowPoint>(
    "follow_point", builder);
}
