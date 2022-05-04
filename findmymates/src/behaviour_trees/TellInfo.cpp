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

#include "behaviour_trees/TellInfo.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>
#include "ros/ros.h"

#include "sound/Speaker.h"
#include "information/Info.h"

namespace behaviour_trees
{
    TellInfo::TellInfo(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
    {
    }
    
    
    void 
    TellInfo::halt()
    {
        ROS_INFO("TellInfo halt");
    }
    
    
    BT::PortsList 
    TellInfo::providedPorts() 
    { 
        return { BT::InputPort<information::Info>("info") }; 
    }
    

    BT::NodeStatus
    TellInfo::tick()
    {
        info_ = getInput<information::Info>("info").value();

        info_speaker_ = "The person at the position " + info_.get_carac(0) + "is " + info_.get_carac(1) + 
                       "the colour of his clothes is " + info_.get_carac(2) + "and he is holding a " + info_.get_carac(3); 

        speaker_.speak(info_speaker_);

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::TellInfo>("tell_info");
}
