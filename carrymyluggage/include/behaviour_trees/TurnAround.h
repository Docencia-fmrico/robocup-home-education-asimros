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

#ifndef BEHAVIOUR_TREES_TURNAROUND_H
#define BEHAVIOUR_TREES_TURNAROUND_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <string>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"


#include "ros/ros.h"

namespace behaviour_trees
{

class TurnAround :  public BT::ActionNodeBase
{
    public:
        explicit TurnAround(const std::string& name);
        
        void halt();

        BT::NodeStatus tick();

    private:
        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener listener;
        ros::NodeHandle nh_;
        ros::Publisher vel_pub_;
        float angspeed_ = 0.4;
        std::string error_;
};

}  // namespace behaviour_trees

#endif  // BEHAVIOUR_TREES_TURNAROUND_H