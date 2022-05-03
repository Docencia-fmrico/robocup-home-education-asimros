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

#include "behaviour_trees/TurnAround.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>
#include "ros/ros.h"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

namespace behaviour_trees
{
    TurnAround::TurnAround(const std::string& name)
    : BT::ActionNodeBase(name, {}),
    listener(buffer)
    {
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

        cmd_.linear.x = 0;
        cmd_.linear.y = 0;
        cmd_.linear.z = 0;
        cmd_.angular.x = 0;
        cmd_.angular.y = 0;
        cmd_.angular.z = 0;
        first_ = true;
    }

    void 
    TurnAround::halt()
    {
        ROS_INFO("TurnAround halt");
    }

    BT::NodeStatus
    TurnAround::tick()
    {
        if (first_) 
        {
            turn_ts_ = ros::Time::now();
            first_ = false;
        }
        if(buffer.canTransform("base_footprint", "person", ros::Time(0), ros::Duration(1.0), &error_))
        {
            ROS_ERROR("I have seen the person again*******************");
            first_ = true;
            return BT::NodeStatus::SUCCESS;
        }

        if((ros::Time::now() - turn_ts_).toSec() < TURNING_TIME)
        {
            ROS_ERROR("turnning*************************************");
            cmd_.angular.z = angspeed_;
            vel_pub_.publish(cmd_);
            return BT::NodeStatus::RUNNING;  
        }
        
        ROS_ERROR("turn: I have not seen the person again********************");
        first_ = true;
        return BT::NodeStatus::FAILURE;
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::TurnAround>("turn_around");
}