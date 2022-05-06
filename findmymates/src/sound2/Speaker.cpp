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
#include "sound2/Speaker.h"

namespace sound
{

  Speaker::Speaker(): n_()
  {
    pub_ = n_.advertise<std_msgs::String>("/speak", 1);
  }

  void
  Speaker::speak(std::string say)
  {
    msg_.data = say;
    pub_.publish(msg_);
  }

}  // namespace sound