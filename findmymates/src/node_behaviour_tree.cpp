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

#include <string>
#include <memory>

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ros/package.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behaviour_tree_nav");
  ros::NodeHandle n;

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("asr_go_point_node"));
  factory.registerFromPlugin(loader.getOSName("asr_localization_node"));
  factory.registerFromPlugin(loader.getOSName("asr_referee_node"));
  factory.registerFromPlugin(loader.getOSName("asr_ask_info_node"));
  factory.registerFromPlugin(loader.getOSName("asr_tell_info_node"));
  factory.registerFromPlugin(loader.getOSName("asr_check_person_node"));
  factory.registerFromPlugin(loader.getOSName("asr_begin_node"));

  auto blackboard = BT::Blackboard::create();

  std::string pkgpath = ros::package::getPath("findmymates");

  std::string xml_file = pkgpath + "/behaviour_trees_xml/tree.xml";

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);
  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  ros::Rate loop_rate(10);

  int count = 0;

  bool finish = false;
  while (ros::ok() && !finish)
  {
    tree.rootNode()->executeTick() ==  BT::NodeStatus::SUCCESS;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}