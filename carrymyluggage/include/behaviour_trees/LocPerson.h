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

#ifndef BEHAVIOUR_TREES_LOCPERSON_H
#define BEHAVIOUR_TREES_LOCPERSON_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <string>

#include "ros/ros.h"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "geometry_msgs/Point.h"

#include "nav_msgs/GetPlan.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

namespace behaviour_trees
{

class LocPerson :  public BT::ActionNodeBase
{
    public:
        explicit LocPerson(const std::string& name, const BT::NodeConfiguration& config);

        void halt();

        static BT::PortsList providedPorts();

        BT::NodeStatus tick();

    private:
        ros::NodeHandle nh_;
        double xgoal_;
        double ygoal_;
        std::string error_;
        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener listener;
        geometry_msgs::TransformStamped map2person_msg;
    	tf2::Stamped<tf2::Transform> map2person;
        ros::ServiceClient client;
        geometry_msgs::PoseStamped start;
        unsigned long calc_index(auto & poses);
};

}  // namespace behaviour_trees

#endif  // BEHAVIOUR_TREES_LOCPERSON_H