#ifndef BBX_FILTER_FILTER_H
#define BBX_FILTER_FILTER_H

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
#include "std_msgs/Int64.h"
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "img_buffer.h"

namespace bbx_filter
{

class Person_filter
{
public:
	Person_filter();
  	void filter_callback(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
	void callback_activation(const std_msgs::Int64::ConstPtr& msg);

private:
	// Pubs and subs
	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_;
  	image_transport::Subscriber image_sub_;
	ros::Publisher bbx_pub;

  	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
            darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;

    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;
    message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;

	// Class atributes
	cv::Point2d pixel_;
	bool first_time;
	Img_buffer buffer;
	bool activated_;

	// Private functions
	void segment_image(cv::Mat img, const darknet_ros_msgs::BoundingBox &box, Segmented_Image & s_img);
	void calc_segment_rgb(cv::Vec3i & segment, cv::Mat img, int x_min, int x_max, int y_min, int y_max); 
	void calc_debug_img(cv::Mat & debug_img, const darknet_ros_msgs::BoundingBox &box, Segmented_Image & s_img);
	void fill_segment_debug(cv::Vec3i & segment, cv::Mat & img, int x_min, int x_max, int y_min, int y_max);

};

} // namespace bbx_filter

#endif // BBX_FILTER_FILTER_H