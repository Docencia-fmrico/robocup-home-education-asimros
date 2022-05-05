#include "bbx_filter/person_filter.h"

namespace bbx_filter
{

Person_filter::Person_filter()
: it_(nh_),
image_sub(nh_, "/camera/rgb/image_raw", 1),
bbx_sub(nh_, "/darknet_ros/bounding_boxes", 1),
sync_bbx(MySyncPolicy_bbx(10), image_sub, bbx_sub),
buffer()
{
	cv::namedWindow("Image debug");
	bbx_pub = nh_.advertise<darknet_ros_msgs::BoundingBoxes>("/bbx_filtered", 1);
    sync_bbx.registerCallback(boost::bind(&Person_filter::filter_callback, this, _1, _2));
	first_time = true;
	activated_ = false;
}


void Person_filter::callback_activation(const std_msgs::Int64::ConstPtr& msg)
{
	if (msg->data == 1) activated_ = true;
}



void Person_filter::filter_callback(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
	if (activated_) {
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
		darknet_ros_msgs::BoundingBoxes bbx_filtered;

		rgb = cv_ptr->image;
		debug_img = cv_imageout->image;

		if (first_time) {
			const auto & box = boxes->bounding_boxes[0];
			Segmented_Image img_init;
			Person_filter::segment_image(rgb, box, img_init);
			buffer.add(img_init);
			first_time = false;
		}
		else {
			for (const auto & box : boxes->bounding_boxes)
			{
				Segmented_Image img_ref;
				Person_filter::segment_image(rgb, box, img_ref);
				if(buffer.stored_similar(img_ref)) {
					Person_filter::calc_debug_img(debug_img, box, img_ref);
					bbx_filtered.bounding_boxes.push_back(box);
					bbx_filtered.image_header = image->header;
					bbx_filtered.header = boxes->header;
					bbx_pub.publish(bbx_filtered);
					buffer.add(img_ref);
					// ROS_INFO("Es la persona objetivo");
					cv::imshow("Image debug", debug_img);
					cv::waitKey(3);
				}
			}
		}
	}
	
}


void Person_filter::segment_image(cv::Mat img, const darknet_ros_msgs::BoundingBox &box, Segmented_Image & s_img) 
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

	for (int y = 0; y < Nsegments; y++) 
	{
		for (int x = 0; x < Nsegments; x++)
		{
			seg_x_start = box.xmin + x * x_inc;
			seg_x_end = seg_x_start + x_inc - 1;
			seg_y_start = box.ymin + y * y_inc;
			seg_y_end = seg_y_start + y_inc - 1;
			if (x == (Nsegments - 1)) seg_x_end += off_x + 1;
			if (y == (Nsegments - 1)) seg_y_end += off_y + 1;
			Person_filter::calc_segment_rgb(s_img.segment[y][x], img, seg_x_start, seg_x_end, seg_y_start, seg_y_end);
		}
	}
}

void Person_filter::calc_segment_rgb(cv::Vec3i & segment, cv::Mat img, int x_min, int x_max, int y_min, int y_max) 
{
	int step = img.step;
	int channels = 3;
	int posdata;
	int pixels = 0;
		
	segment[0] = 0;
	segment[1] = 0;
	segment[2] = 0;  
	for (int i = y_min; i < y_max; i++ )
	{
      	for (int j = x_min; j < x_max; j++ ) 
		{
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

void Person_filter::calc_debug_img(cv::Mat & debug_img, const darknet_ros_msgs::BoundingBox &box, Segmented_Image & s_img) 
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

	for (int y = 0; y < Nsegments; y++) 
	{
		for (int x = 0; x < Nsegments; x++)
		{
			seg_x_start = box.xmin + x * x_inc;
			seg_x_end = seg_x_start + x_inc - 1;
			seg_y_start = box.ymin + y * y_inc;
			seg_y_end = seg_y_start + y_inc - 1;
			if (x == (Nsegments - 1)) seg_x_end += off_x + 1;
			if (y == (Nsegments - 1)) seg_y_end += off_y + 1;
			Person_filter::fill_segment_debug(s_img.segment[y][x], debug_img, seg_x_start, seg_x_end, seg_y_start, seg_y_end);
		}
	}
}	

void Person_filter::fill_segment_debug(cv::Vec3i & segment, cv::Mat & img, int x_min, int x_max, int y_min, int y_max) 
{
	int step = img.step;
	int channels = 3;
	int posdata;

	for (int i = y_min; i < y_max; i++ )
	{
      	for (int j = x_min; j < x_max; j++ ) 
		{
        	posdata = i * step + j * channels;
			img.data[posdata] = segment[0];
			img.data[posdata + 1] = segment[1];
			img.data[posdata + 2] = segment[2];
      	}
  	}
}

} // namespace bbx_filter
