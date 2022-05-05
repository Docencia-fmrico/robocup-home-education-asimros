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

#include "behaviour_trees/AskInfo.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>
#include "ros/ros.h"

#include "sound/Listener.h"
#include "sound/Speaker.h"
#include "information/Info.h"

namespace behaviour_trees
{
    AskInfo::AskInfo(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
    {
        first_ = true;
        speak_ = true;
    }
    
    
    void 
    AskInfo::halt()
    {
        ROS_INFO("AskInfo halt");
    }
    
    
    BT::PortsList 
    AskInfo::providedPorts() 
    { 
        return { BT::OutputPort<information::Info>("info"), BT::InputPort<int>("count") }; 
    }
    

    BT::NodeStatus
    AskInfo::tick()
    {
        std::string questions[3] = {"What is your name?", "What is the color of your clothes?", "Which object are you holding?"};
        int i = 0;
        
        if(first_)
        {
            info_.set_pos(getInput<int>("count").value());
            speak_ts_ = ros::Time::now();
            first_ = false;
        }

        if(speak_)
        {
            speaker_.speak(questions[i]);
            speak_ = false;
        }

        if((ros::Time::now() - speak_ts_).toSec() > SPEAKING_TIME)
        {
            listener_.listen();
            std::string answer = listener_.answer();

            if(!answer.compare("false"))
            {
                i++;
                info_.set_carac(answer, i);
                speak_ = true;

                if(i == 3)
                {
                    ROS_ERROR("He terminado de rellenar info");
                    first_ = true;
                    setOutput<information::Info>("info", info_);
                    return BT::NodeStatus::SUCCESS;
                }
            }
            else
            {
                speak_ = true;
            }
        }

        return BT::NodeStatus::RUNNING;
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::AskInfo>("ask_info");
}

