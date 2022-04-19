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
    FollowPoint::FollowPoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
    {
    }

    void 
    FollowPoint::halt()
    {
        ROS_INFO("FollowPoint halt");
    }

    BT::PortsList 
    FollowPoint::providedPorts() 
    { 
        return { BT::InputPort<move_base_msgs::MoveBaseGoal>("goal_nav") }; 
    }

    BT::NodeStatus
    FollowPoint::tick()
    {
        BT::Optional<move_base_msgs::MoveBaseGoal> goal = getInput<move_base_msgs::MoveBaseGoal>("goal_nav"); 
        
        if(!goal)
        {
            throw BT::RuntimeError("missing required input [goal_nav]: ", goal.error());
        }

        nav_client_.doWork(goal.value());

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::FollowPoint>("follow_point");
}