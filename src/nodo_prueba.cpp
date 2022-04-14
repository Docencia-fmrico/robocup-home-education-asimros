#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "detect_case/SideCase.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nodo_prueba");

    detect_case::SideCase side;

    ros::spin();

    return 0;
}