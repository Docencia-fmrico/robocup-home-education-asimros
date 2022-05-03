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
#include "detect_case/SideCase.h"
#include "geometry_msgs/Twist.h"

#include "sound/Speaker.h"

namespace behaviour_trees
{

class ChooseSide : public BT::ActionNodeBase
{
    public:
        explicit ChooseSide(const std::string& name);
        
        void halt();
        
        BT::NodeStatus tick();

    private:
        ros::NodeHandle nh_;
        detect_case::SideCase case_;
        bool first_;

		geometry_msgs::Twist cmd_;
		ros::Publisher vel_pub_;

		bool know_side_;
		float angspeed_;

		static constexpr double TURNING_TIME = 2.0;
		ros::Time turn_ts_;
        sound::Speaker speaker_;
};

}  // namespace behaviour_trees

#endif  // BEHAVIOUR_TREES_CHOOSESIDE_H