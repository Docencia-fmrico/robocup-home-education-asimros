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

#include "behaviour_trees/CheckPerson.h"
#include "detect_person/DetectPerson.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>
#include "ros/ros.h"

namespace behaviour_trees
{
    CheckPerson::CheckPerson(const std::string& name)
    : BT::ActionNodeBase(name, {})
    {
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);

        cmd_.linear.x = 0;
        cmd_.linear.y = 0;
        cmd_.linear.z = 0;
        cmd_.angular.x = 0;
        cmd_.angular.y = 0;
        cmd_.angular.z = 0;
        first_ = true;
    }

    void 
    CheckPerson::halt()
    {
        ROS_INFO("CheckPerson halt");
    }

    BT::NodeStatus
    CheckPerson::tick()
    {
        if (first_) 
        {
            turn_ts_ = ros::Time::now();
            first_ = false;
        }

        if(person_.has_seen()){
            ROS_ERROR("He visto a la persona");
            first_ = true;
            person_.lost();
            return BT::NodeStatus::SUCCESS; 
        }

        if((ros::Time::now() - turn_ts_).toSec() < TURNING_TIME)
        {
            ROS_ERROR("turnning");
            cmd_.angular.z = angspeed_;
            vel_pub_.publish(cmd_);
            return BT::NodeStatus::RUNNING;  
        }
        
        ROS_ERROR("There isn't person");
        first_ = true;
        return BT::NodeStatus::FAILURE;
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::CheckPerson>("check_person");
}