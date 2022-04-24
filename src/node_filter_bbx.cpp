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


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <darknet_ros_msgs/BoundingBoxes.h>

class BbxConverter
{
public:
  	BbxConverter()
	: it_(nh_),
  	image_sub(nh_, "/usb_cam/image_raw", 1),
	bbx_sub(nh_, "/darknet_ros/bounding_boxes", 1),
	sync_bbx(MySyncPolicy_bbx(10), image_sub, bbx_sub)
  {
    sync_bbx.registerCallback(boost::bind(&BbxConverter::filter_callback, this, _1, _2));
	first_time = true;
  }


  void filter_callback(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat hsv;
    cv::cvtColor(cv_ptr->image, hsv, CV_RGB2HSV);

	if (first_time) {
		const auto & box = boxes->bounding_boxes[0];
		calc_hsv_mean(hsv, box, hsvReference);
		ROS_INFO("H = %d S = %d V = %d", hsvReference[0], hsvReference[1], hsvReference[2]);
		first_time = false;
	}
	else {
		for (const auto & box : boxes->bounding_boxes) {
        	cv::Vec3i hsvMean;
			calc_hsv_mean(hsv, box, hsvMean);
			ROS_INFO("H = %d S = %d V = %d", hsvMean[0] - hsvReference[0], hsvMean[1] - hsvReference[1], hsvMean[2] - hsvReference[2]);

    	}
	}
  }

private:
	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_;
  	image_transport::Subscriber image_sub_;

  	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
            darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;

    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;
    message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;
	cv::Point2d pixel_;
	bool first_time;
	cv::Vec3i hsvReference;
	
	void calc_hsv_mean(cv::Mat img, const darknet_ros_msgs::BoundingBox &box, cv::Vec3i & results) 
	{
		int width;
		int height;
		int step = img.step;
		int channels = 3;
		int posdata;

		results[0] = 0;
		results[1] = 0;
		results[2] = 0;
		width = box.xmax - box.xmin;
        height = box.ymax - box.ymin;    
		for (int i = box.ymin; i < height; i++ ){
      		for (int j = box.xmin; j < width; j++ ) {
        		posdata = i * step + j * channels;
				results[0] = results[0] + img.data[posdata];
				results[1] = results[1] + img.data[posdata+1];
				results[2] = results[2] + img.data[posdata+2];
      		}
  		}
		results[0] = results[0]  / (height * width);
		results[1] = results[1]  / (height * width);
		results[2] = results[2]  / (height * width);
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bbx_filter");
  BbxConverter ic;
  ros::spin();
  return 0;
}
