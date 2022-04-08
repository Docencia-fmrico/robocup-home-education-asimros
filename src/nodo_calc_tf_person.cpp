#include "ros/ros.h"
#include "person_tf/Tf_calc.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd_tf");
  person_tf::Tf_calc calculator;
  ros::spin();
  return 0;
}