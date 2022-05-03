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

#include "detect_case/SideCase.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "ros/ros.h"

namespace detect_case
{

SideCase::SideCase()
: image_depth_sub(nh, "/camera/depth/image_raw", 1),
bbx_sub(nh, "/darknet_ros/bounding_boxes", 1),
sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub)
{
    sync_bbx.registerCallback(boost::bind(&SideCase::callback_bbx, this, _1, _2));
    read_ts_ = ros::Time::now();
    side_ = 0;
    dif_ = 0;
}

void
SideCase::callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
    int px;
    cv_bridge::CvImagePtr img_ptr_depth;
    try
    {
        img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    for (const auto & box : boxes->bounding_boxes)
    {
        px = (box.xmax + box.xmin) / 2;
    }
    if((ros::Time::now() - read_ts_).toSec() < 0.1){ //first time
        px_= px;
    }
    if((ros::Time::now() - read_ts_).toSec() < READ_TIME){
        dif_ = dif_ + px - px_ ;
        side_= 0;
    }
    else{
        if (dif_ < 0){
            side_ = 1;
        }
        else{
            side_ = 2;
        }
    }
    px_= px;
    
}

}  // namespace detect_case
