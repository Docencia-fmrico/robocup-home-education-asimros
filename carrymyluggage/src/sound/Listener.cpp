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
    sub_ = n_.subscribe("message", 1, &Listener::messageCallback, this);
    warn_;
    finished_ = false;
    msg_ = "listen bro";
  }

  void
  Listener::messageCallback(const std_msgs::String::ConstPtr& msg)
  {
    if(msg->data.compare("true"))
    {
        finished_ = true;
        warn_ = false;
    } else
    {
      finished_ = true;
      warn_ = true;
      warning_ts_ = ros::Time::now().toSec();
    }
  }

}  // namespace sound
