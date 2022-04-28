#include "bbx_filter/img_buffer.h"

namespace bbx_filter
{

Img_buffer::Img_buffer(): index(0), nimages(0), times_nd(0) {} 

// Adds a segmented image to the buffer
void Img_buffer::add(Segmented_Image img) 
{
	if (index >= Memory) index = 0;
	if (nimages < Memory) nimages++;
	images[index] = img;
	index++;
}

// Checks if an stored image has enough similarity with the given s_img
bool Img_buffer::stored_similar(Segmented_Image & s_img) 
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

// Calcs the number of similar segments between two s_imgs, considered similar if diff < thres
int Img_buffer::calc_nsims(Segmented_Image & s_img, Segmented_Image & ref_img) 
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

// Applies an exponential backoff for the number of segments needed to be considered similar
bool Img_buffer::exp_backoff(int nsims) 
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

} // namespace img_buffer