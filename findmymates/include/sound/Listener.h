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

#ifndef SOUND_LISTENER_H
#define SOUND_LISTENER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

namespace sound
{

class Listener
{
public:
  Listener();

  void messageCallback(const std_msgs::String::ConstPtr& msg);

  bool get_finished() {return finished_;}
  void listen();
  void warn();
  bool recived() {return recived_;}

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std_msgs::String msg_;

  bool recived_;

};

}  // namespace sound

#endif // SOUND_LISTENER_H
