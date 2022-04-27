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

enum {Nsegments = 6, Memory = 8};

typedef struct
{
  cv::Vec3i segment[Nsegments][Nsegments]; 
}
Segmented_Image;

class ImgBuffer
{
public:
	ImgBuffer(): index(0), nimages(0), times_nd(0)
	{} 
	void add(Segmented_Image img) 
	{
		if (index >= Memory) index = 0;
		if (nimages < Memory) nimages++;
		images[index] = img;
		index++;
	}

	bool stored_similar(Segmented_Image & s_img) 
	{
		int nsims;
		bool similar;
		for (int i = 0; i < nimages; i++) {
			nsims = calc_nsims(s_img, images[i]);
			similar = exp_backoff(nsims);
			if (similar) {
				return similar;
				times_nd = 0;
			}
		}
		times_nd++;
		return false;
	}

private:
	int index;
	int nimages;
	int times_nd;
	Segmented_Image images[Memory];

	// Needs to be standarized and probably redone
	bool exp_backoff(int nsims) 
	{	
		bool similar;
		int total_segments = Nsegments * Nsegments;
		int lev1 = int(total_segments * 0.9);
		int lev2 = int(total_segments * 0.8);
		int lev3 = int(total_segments * 0.7);
		int lev4 = int(total_segments * 0.6);

		if (times_nd < 100) similar = nsims >= lev1;
		else if (times_nd > 100 && times_nd < 500) similar = nsims >= lev2;
		else if (times_nd > 500 && times_nd < 2000) similar = nsims >= lev3;
		else similar = nsims >= lev4;

		return similar;
	}

	int calc_nsims(Segmented_Image & s_img, Segmented_Image & ref_img) 
	{
		int b_thres = 30;
		int g_thres = 30;
		int r_thres = 30;

		int sim_segments;
		int nsegments = Nsegments * Nsegments;

		cv::Vec3i diff;
		cv::Vec3i vPerson;
		cv::Vec3i vRef;
		
		sim_segments = 0;
		for (int y = 0; y < Nsegments; y++){
			for (int x = 0; x < Nsegments; x++){
				vPerson = s_img.segment[y][x];
				vRef = ref_img.segment[y][x];
				diff[0] = abs(vPerson[0]- vRef[0]);
				diff[1] = abs(vPerson[1]- vRef[1]);
				diff[2] = abs(vPerson[2]- vRef[2]);
				if (diff[0] <= b_thres && diff[1] <= g_thres && diff[2] <= r_thres) sim_segments++;
			}
		}
		return sim_segments;
	}
};

class BbxConverter
{
public:
  	BbxConverter()
	: it_(nh_),
  	image_sub(nh_, "/usb_cam/image_raw", 1),
	bbx_sub(nh_, "/darknet_ros/bounding_boxes", 1),
	sync_bbx(MySyncPolicy_bbx(10), image_sub, bbx_sub)
  {
	cv::namedWindow("Image debug");
    sync_bbx.registerCallback(boost::bind(&BbxConverter::filter_callback, this, _1, _2));
	first_time = true;
  }


  void filter_callback(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
  {
    cv_bridge::CvImagePtr cv_ptr, cv_imageout;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
	  cv_imageout = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat rgb;
	cv::Mat debug_img;

	rgb = cv_ptr->image;
	debug_img = cv_imageout->image;

	if (first_time) {
		const auto & box = boxes->bounding_boxes[0];
		Segmented_Image img_init;
		segment_image(rgb, box, img_init);
		buffer.add(img_init);
		first_time = false;
	}
	else {
		for (const auto & box : boxes->bounding_boxes)
    	{
			Segmented_Image img_ref;
			segment_image(rgb, box, img_ref);
			if(buffer.stored_similar(img_ref)) {
				calc_debug_img(debug_img, box, img_ref);
				buffer.add(img_ref);
				cv::imshow("Image debug", debug_img);
				cv::waitKey(3);
			}
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

	ImgBuffer buffer;
	
	void segment_image(cv::Mat img, const darknet_ros_msgs::BoundingBox &box, Segmented_Image & s_img) 
	{
		int x_size;
		int y_size;
		int off_x;
		int off_y;
		int x_inc;
		int y_inc;
		int seg_x_start;
		int seg_x_end;
		int seg_y_start;
		int seg_y_end;
		int nsegment;

		x_size = box.xmax - box.xmin;
		y_size = box.ymax - box.ymin;

		off_x = x_size % Nsegments;
		off_y = y_size % Nsegments;

		x_inc = x_size / Nsegments;
		y_inc = y_size / Nsegments;

		for (int y = 0; y < Nsegments; y++) {
			for (int x = 0; x < Nsegments; x++){
				seg_x_start = box.xmin + x * x_inc;
				seg_x_end = seg_x_start + x_inc - 1;
				seg_y_start = box.ymin + y * y_inc;
				seg_y_end = seg_y_start + y_inc - 1;
				if (x == (Nsegments - 1)) seg_x_end += off_x + 1;
				if (y == (Nsegments - 1)) seg_y_end += off_y + 1;
				calc_segment_rgb(s_img.segment[y][x], img, seg_x_start, seg_x_end, seg_y_start, seg_y_end);
			}
		}
	}

	void calc_debug_img(cv::Mat & debug_img, const darknet_ros_msgs::BoundingBox &box, Segmented_Image & s_img) 
	{
		int x_size;
		int y_size;
		int off_x;
		int off_y;
		int x_inc;
		int y_inc;
		int seg_x_start;
		int seg_x_end;
		int seg_y_start;
		int seg_y_end;
		int nsegment;

		x_size = box.xmax - box.xmin;
		y_size = box.ymax - box.ymin;

		off_x = x_size % Nsegments;
		off_y = y_size % Nsegments;

		x_inc = x_size / Nsegments;
		y_inc = y_size / Nsegments;

		for (int y = 0; y < Nsegments; y++) {
			for (int x = 0; x < Nsegments; x++){
				seg_x_start = box.xmin + x * x_inc;
				seg_x_end = seg_x_start + x_inc - 1;
				seg_y_start = box.ymin + y * y_inc;
				seg_y_end = seg_y_start + y_inc - 1;
				if (x == (Nsegments - 1)) seg_x_end += off_x + 1;
				if (y == (Nsegments - 1)) seg_y_end += off_y + 1;
				fill_segment_debug(s_img.segment[y][x], debug_img, seg_x_start, seg_x_end, seg_y_start, seg_y_end);
			}
		}
	}	

	void fill_segment_debug(cv::Vec3i & segment, cv::Mat & img, int x_min, int x_max, int y_min, int y_max) 
	{
		int step = img.step;
		int channels = 3;
		int posdata;

		for (int i = y_min; i < y_max; i++ ){
      		for (int j = x_min; j < x_max; j++ ) {
        		posdata = i * step + j * channels;
				img.data[posdata] = segment[0];
				img.data[posdata + 1] = segment[1];
				img.data[posdata + 1] = segment[2];
      		}
  		}

		// cv::imshow("Image debug", cv_imageout->image);
	}


	void calc_segment_rgb(cv::Vec3i & segment, cv::Mat img, int x_min, int x_max, int y_min, int y_max) 
	{
		int step = img.step;
		int channels = 3;
		int posdata;
		int pixels = 0;
		
		segment[0] = 0;
		segment[1] = 0;
		segment[2] = 0;  
		for (int i = y_min; i < y_max; i++ ){
      		for (int j = x_min; j < x_max; j++ ) {
        		posdata = i * step + j * channels;
				segment[0] += img.data[posdata];
				segment[1] += img.data[posdata+1];
				segment[2] += img.data[posdata+2];
				pixels++;
      		}
  		}
		segment[0] /= pixels;
		segment[1] /= pixels;
		segment[2] /= pixels;
	}

	void debug_s_image(Segmented_Image & s_img) 
	{
		int b;
		int g;
		int r;

		for (int y = 0; y < Nsegments; y++){
			for (int x = 0; x < Nsegments; x++){
				ROS_INFO("Segment (%d,%d)", y, x);
				b = s_img.segment[y][x][0];
				g = s_img.segment[y][x][1];
				r = s_img.segment[y][x][2];
				ROS_INFO("B = %d, G = %d, R = %d", b, g, r);
			}
		}
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bbx_filter");
  BbxConverter ic;
  ros::spin();
  return 0;
}
