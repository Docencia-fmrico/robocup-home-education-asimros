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
  ros::init(argc, argv, "behaviour_tree_nav_init");
  ros::NodeHandle n;

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;
  BT::BehaviorTreeFactory factory2;
  BT::SharedLibrary loader2;

  factory.registerFromPlugin(loader.getOSName("asr_wait_for_person_node"));
  factory.registerFromPlugin(loader.getOSName("asr_choose_side_node"));
  factory.registerFromPlugin(loader.getOSName("asr_follow_point_node"));
  factory.registerFromPlugin(loader.getOSName("asr_ask_for_case_node"));
  factory.registerFromPlugin(loader.getOSName("asr_arena_node"));
  factory.registerFromPlugin(loader.getOSName("asr_start_node"));

  factory2.registerFromPlugin(loader2.getOSName("asr_loc_person_node"));
  factory2.registerFromPlugin(loader2.getOSName("asr_turn_around_node"));
  factory2.registerFromPlugin(loader2.getOSName("asr_request_come_closer_node"));
  factory2.registerFromPlugin(loader2.getOSName("asr_follow_point_node"));
  factory2.registerFromPlugin(loader2.getOSName("asr_arena_node"));
  factory2.registerFromPlugin(loader2.getOSName("asr_have_finished_node"));
  

  auto blackboard = BT::Blackboard::create();

  std::string pkgpath = ros::package::getPath("carrymyluggage");

  std::string xml_file = pkgpath + "/behaviour_trees_xml/tree_nav_init.xml";
  std::string xml_file2 = pkgpath + "/behaviour_trees_xml/tree_nav.xml";

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);
  BT::Tree tree2 = factory2.createTreeFromFile(xml_file2, blackboard);

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  ros::Rate loop_rate(10);

  int count = 0;

  bool finish1 = false;
  bool finish2 = false;
  while (ros::ok() && !finish2)
  {
    if(!finish1)
    {
      finish1 = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    }
    else
    {
      ROS_ERROR("HACIENDO TICK AL SEGUNDO ARBOLLLL");
      finish2 = tree2.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}