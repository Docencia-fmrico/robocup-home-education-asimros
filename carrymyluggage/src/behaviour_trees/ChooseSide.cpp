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

#include "sound/Speaker.h"

#include "ros/ros.h"

namespace behaviour_trees
{

    ChooseSide::ChooseSide(const std::string& name)
    : BT::ActionNodeBase(name, {})
    {
      know_side_ = false;
      vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);

      cmd_.linear.x = 0.0;
      cmd_.linear.y = 0.0;
      cmd_.linear.z = 0.0;
      cmd_.angular.x = 0.0;
      cmd_.angular.y = 0.0;
      cmd_.angular.z = 0.0;
      first_ = true;
    }

    void 
    ChooseSide::halt()
    {
        ROS_INFO("ChooseSide halt");
    }

    BT::NodeStatus
    ChooseSide::tick()
    {
      if(first_) {
        speaker_.speak("Please point the bag");
        first_ = false;
      }

      if(!know_side_ &&  case_.get_side() != 0)
      {
        ROS_ERROR("He elegido un lado******************************");
        know_side_ = true;
        turn_ts_ = ros::Time::now();

        if(case_.get_side() == 1)
        {
          angspeed_ = 0.4;
        }
        else
        {
          angspeed_ = -0.4;
        }
      }
      else 
      {
        ROS_ERROR("NO HE ELEGIDO UN LADO KKKKK******************************");
      }

      if(know_side_)
      {
        if((ros::Time::now() - turn_ts_).toSec() < TURNING_TIME)
        {
          ROS_ERROR("I AM TURNING******************************");
          cmd_.angular.z = angspeed_;
          vel_pub_.publish(cmd_);
        }
        else
        {
          ROS_ERROR("I HAVE TURNED ******************************************************");
          return BT::NodeStatus::SUCCESS;
        }
      }

      return BT::NodeStatus::RUNNING;
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::ChooseSide>("choose_side");
}