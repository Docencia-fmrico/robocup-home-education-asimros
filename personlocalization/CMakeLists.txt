cmake_minimum_required(VERSION 2.8.3)
project(personlocalization)
set(CMAKE_CXX_FLAGS "-std=c++17")

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  image_geometry
  roslint
  sensor_msgs
  tf
  cv_bridge
  image_transport
  darknet_ros_msgs
  geometry_msgs
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  image_geometry
  sensor_msgs
  tf
  cv_bridge
  image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/person_tf/Tf_calc.cpp
)

add_executable(node_calc_tf_person src/node_calc_tf_person.cpp)
target_link_libraries(node_calc_tf_person ${PROJECT_NAME}  ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_executable(node_filter_bbx src/node_filter_bbx.cpp)
target_link_libraries(node_filter_bbx ${PROJECT_NAME} ${catkin_LIBRARIES})

roslint_cpp(
  src/node_calc_tf_person.cpp
  src/node_filter_bbx.cpp
  include/person_tf/Tf_calc.h 
  src/person_tf/Tf_calc.cpp
)
