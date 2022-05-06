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

#ifndef BEHAVIOUR_TREES_ASKINFO_H
#define BEHAVIOUR_TREES_ASKINFO_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <string>

#include "ros/ros.h"
#include "sound2/Speaker.h"
#include "sound2/Listener.h"
#include "information/Info.h"

namespace behaviour_trees
{

    class AskInfo : public BT::ActionNodeBase
    {
        public:
            explicit AskInfo(const std::string& name, const BT::NodeConfiguration& config);

            void halt();

            static BT::PortsList providedPorts();

            BT::NodeStatus tick();

        private:
            ros::NodeHandle nh_;
            sound2::Speaker speaker_;
            information::Info info_;
            sound2::Listener listener_;

            static constexpr double SPEAKING_TIME = 3.0;
            ros::Time speak_ts_;

            bool speak_;
            bool first_;
    };

}  // namespace behaviour_trees

#endif  // BEHAVIOUR_TREES_ASKINFO_H