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

#ifndef PERSON_TF_TF_CALC_H
#define PERSON_TF_TF_CALC_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <image_geometry/pinhole_camera_model.h>

#include "ros/ros.h"

namespace person_tf
{

class Tf_calc
{
  public:
    Tf_calc();
    void callback_tf(const sensor_msgs::ImageConstPtr& image,
                      const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
    
    void callback_caminfo(const sensor_msgs::CameraInfoConstPtr& msg);

  private:
    ros::NodeHandle nh_;

    cv::Point2d pixel_;
    cv::Point3d xyz_;
    ros::Subscriber sub_cam_;
    image_geometry::PinholeCameraModel cammodel_;
    double dist_;
    ros::Time boundingtime_;
	bool detected_;

    std::string objectFrameId_;
    std::string workingFrameId_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
            darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;

    message_filters::Subscriber<sensor_msgs::Image> image_depth_sub;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;
    message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;

    tf::TransformBroadcaster tfBroadcaster_;
    tf::StampedTransform transform_;
};

}  // namespace tf_calc

#endif  // PERSON_TF_TF_CALC_H
