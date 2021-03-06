cmake_minimum_required(VERSION 2.8.3)
project(carrymyluggage)
set(CMAKE_CXX_FLAGS "-std=c++17")

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  image_geometry
  behaviortree_cpp_v3
  nav_msgs
  roslint
  sensor_msgs
  tf
  cv_bridge
  image_transport
  darknet_ros_msgs
  geometry_msgs

  dialogflow_ros_msgs
  std_srvs
  sound_play
  actionlib
  actionlib_msgs
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  nav_msgs
  image_geometry
  behaviortree_cpp_v3
  sensor_msgs
  tf
  cv_bridge
  image_transport

	LIBRARIES gb_dialog
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

add_library(asr_wait_for_person_node SHARED src/behaviour_trees/WaitForPerson.cpp)
add_library(asr_choose_side_node SHARED src/behaviour_trees/ChooseSide.cpp)
add_library(asr_follow_point_node SHARED src/behaviour_trees/FollowPoint.cpp)
add_library(asr_ask_for_case_node SHARED src/behaviour_trees/AskForCase.cpp)
add_library(asr_arena_node SHARED src/behaviour_trees/Arena.cpp)
add_library(asr_loc_person_node SHARED src/behaviour_trees/LocPerson.cpp)
add_library(asr_turn_around_node SHARED src/behaviour_trees/TurnAround.cpp)
add_library(asr_request_come_closer_node SHARED src/behaviour_trees/RequestComeCloser.cpp)
add_library(asr_have_finished_node SHARED src/behaviour_trees/HaveFinished.cpp)
add_library(asr_start_node SHARED src/behaviour_trees/Start.cpp)

list(APPEND plugin_libs
  asr_choose_side_node
  asr_wait_for_person_node
  asr_follow_point_node
  asr_ask_for_case_node
  asr_loc_person_node
  asr_request_come_closer_node
  asr_turn_around_node
  asr_arena_node
  asr_have_finished_node
  asr_start_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_library(${PROJECT_NAME}
  src/detect_case/SideCase.cpp
  src/sound/Listener.cpp
  src/sound/Speaker.cpp
)

add_executable(node_behaviour_tree_nav_init src/node_behaviour_tree_nav_init.cpp)
target_link_libraries(node_behaviour_tree_nav_init ${PROJECT_NAME} ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})
add_executable(node_behaviour_tree_nav src/node_behaviour_tree_nav.cpp)
target_link_libraries(node_behaviour_tree_nav ${PROJECT_NAME} ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})
add_executable(node_bt_together src/node_bt_together.cpp)
target_link_libraries(node_bt_together ${PROJECT_NAME} ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})


roslint_cpp(
  src/behaviour_trees/WaitForPerson.cpp
  include/behaviour_trees/WaitForPerson.h
  src/behaviour_trees/ChooseSide.cpp
  include/behaviour_trees/ChooseSide.h
  src/detect_case/SideCase.cpp
  include/detect_case/SideCase.h
  src/behaviour_trees/FollowPoint.cpp
  include/behaviour_trees/FollowPoint.h
  src/behaviour_trees/AskForCase.cpp
  include/behaviour_trees/AskForCase.h
  src/behaviour_trees/Arena.cpp
  include/behaviour_trees/Arena.cpp
  include/behaviour_trees/BTNavAction.h
  src/node_behaviour_tree_nav_init.cpp
  src/behaviour_trees/TurnAround.cpp
  include/behaviour_trees/TurnAround.cpp
  src/behaviour_trees/LocPerson.cpp
  include/behaviour_trees/LocPerson.h
  src/behaviour_trees/RequestComeCloser.cpp
  include/behaviour_trees/RequestComeCloser.h
  src/behaviour_trees/HaveFinished.cpp
  include/behaviour_trees/HaveFinished.h
  src/behaviour_trees/Start.cpp
  include/behaviour_trees/Start.h
  src/node_behaviour_tree_nav.cpp
  src/node_bt_together.cpp
  src/sound/Listener.cpp
  include/sound/Listener.h
  src/sound/Speaker.cpp
  include/sound/Speaker.h
)
