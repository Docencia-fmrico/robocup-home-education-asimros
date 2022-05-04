cmake_minimum_required(VERSION 2.8.3)
project(findmymates)
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

add_library(asr_go_point_node SHARED src/behaviour_trees/GoPoint.cpp)
add_library(asr_referee_node SHARED src/behaviour_trees/Referee.cpp)
add_library(asr_localization_node SHARED src/behaviour_trees/Localization.cpp)
add_library(asr_check_person_node SHARED src/behaviour_trees/CheckPerson.cpp)

list(APPEND plugin_libs
  asr_go_point_node
  asr_referee_node
  asr_localization_node
  asr_check_person_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_library(${PROJECT_NAME}
  src/detect_person/DetectPerson.cpp
)

add_executable(node_behaviour_tree src/node_behaviour_tree.cpp)
target_link_libraries(node_behaviour_tree ${PROJECT_NAME} ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

roslint_cpp(
  src/behaviour_trees/GoPoint.cpp
  include/behaviour_trees/GoPoint.h
  src/behaviour_trees/Referee.cpp
  include/behaviour_trees/Referee.h
  src/behaviour_trees/Localization.cpp
  include/behaviour_trees/Localization.h
  src/behaviour_trees/CheckPerson.cpp
  include/behaviour_trees/CheckPerson.h
  include/behaviour_trees/DetectPerson.h
  src/behaviour_trees/DetectPerson.cpp
)
