cmake_minimum_required(VERSION 2.8.3)
project(sound)
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


foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


add_executable(listener_df_node src/nodes/listenerDF.cpp)
add_dependencies(listener_df_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries(listener_df_node ${catkin_LIBRARIES} gb_dialog)

add_executable(speaker_df_node src/nodes/speakerDF.cpp)
add_dependencies(speaker_df_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries(speaker_df_node ${catkin_LIBRARIES} gb_dialog)


add_executable(test_df_node src/nodes/testDF.cpp)
add_dependencies(test_df_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries(test_df_node ${catkin_LIBRARIES} gb_dialog)

roslint_cpp(
  src/nodes/listenerDF.cpp
  src/nodes/speakerDF.cpp
)
