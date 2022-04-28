#ifndef BBX_FILTER_IMG_BUFFER_H
#define BBX_FILTER_IMG_BUFFER_H

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

namespace bbx_filter
{

enum {Nsegments = 6, Memory = 8};

typedef struct
{
  cv::Vec3i segment[Nsegments][Nsegments]; 
}
Segmented_Image;


class Img_buffer
{
public:
	Img_buffer();
	void add(Segmented_Image img);
	bool stored_similar(Segmented_Image & s_img);

private:
	int index;
	int nimages;
	int times_nd;
	Segmented_Image images[Memory];

	// Private functions
	bool exp_backoff(int nsims);
	int calc_nsims(Segmented_Image & s_img, Segmented_Image & ref_img);
};

} // namespace bbx_filter

#endif