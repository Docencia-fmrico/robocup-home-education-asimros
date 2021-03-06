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

#ifndef DETECT_PERSON_DETECT_PERSON_H
#define DETECT_PERSON_DETECT_PERSON_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "ros/ros.h"

namespace detect_person
{

class DetectPerson
{
    public:
        DetectPerson();

        void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
        bool has_seen() {return seen_;}
        void lost() {seen_ = false;}

    private:
        ros::NodeHandle nh;

        bool seen_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
        message_filters::Subscriber<sensor_msgs::Image> image_depth_sub;
        message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;
        message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;
};

}  // namespace detect_person

#endif  //DETECT_PERSON_DETECT_PERSON_H