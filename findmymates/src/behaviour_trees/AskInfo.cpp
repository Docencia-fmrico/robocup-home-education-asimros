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

        info_.set_pos(getInput<int>("count").value());

        for(int i = 0; i < 3; i++)
        {
            speaker_.speak(questions[i]);
            //esperar -> se pisa con el listener
        
            listener_.listen();
            std::string answer = listener_.answer();

            //si recibe algo
            info_.set_carac(answer, i + 1);
            //sino
            // pedirle que lo repita y volver a ponerlo a escuchar
            //equivalente a reastar 1 a i
        }

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace behaviour_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behaviour_trees::AskInfo>("ask_info");
}

