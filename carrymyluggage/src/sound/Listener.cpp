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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include "sound/Listener.h"

namespace sound
{

  Listener::Listener(): n_()
  {
    sub_ = n_.subscribe("/answer", 1, &Listener::messageCallback, this);
    pub_ = n_.advertise<std_msgs::String>("/listen", 1);
    finished_ = false;
    warn_ = false;
    recived_ = false;
    msg_.data = "listen bro";
  }

  
  void
  Listener::listen()
  {
    pub_.publish(msg_);    
  }

  void
  Listener::messageCallback(const std_msgs::String::ConstPtr& msg)
  {
    if(msg->data.compare("true"))
    {
      finished_ = true;
      recived_ = true;
      warn_ = false;
    } 
    else
    {
      recived_ = true;
      warn_ = true;
      warning_ts_ = (ros::Time::now()).toSec();
    }
  }

  void
  Listener::warn()
  {
    if(warn_ && ((warning_ts_ - ros::Time::now().toSec()) >= WARNING_TIME))
    {
      pub_.publish(msg_);
    }
  }

}  // namespace sound
