
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include <iostream>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

#include "rectify_images.h"
#include "parameters.h"


static cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,16,3);
static cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create( 16*3, 21 ); 




cv::Mat rectifyImages(const cv::Mat& imageLeft, const cv::Mat& imageRight, cv::Mat& imageLeft_l, cv::Mat& imageRight_r, cv::Mat& Q, bool draw){


	
	
	cv::Mat R1, R2, P1, P2;
	cv::Mat rmap[2][2];

	cv::stereoRectify(CM[0], DC[0],
                  CM[1], DC[1],
                  IMAGES_SIZE, STEREO_R, STEREO_T, R1, R2, P1, P2, Q,
                  cv::CALIB_ZERO_DISPARITY, 1, IMAGES_SIZE);

	cv::initUndistortRectifyMap(CM[0], DC[0], R1, P1, IMAGES_SIZE, CV_16SC2, rmap[0][0], rmap[0][1]);
	cv::initUndistortRectifyMap(CM[1], DC[1], R2, P2, IMAGES_SIZE, CV_16SC2, rmap[1][0], rmap[1][1]);

	cv::remap(imageLeft, imageLeft_l, rmap[0][0], rmap[1][1], cv::INTER_LINEAR);
	cv::remap(imageRight, imageRight_r, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);


	//DRAW RECTIFIED IMAGE
	if (draw){
		cv::Mat pair;
		pair.create(IMAGES_SIZE.height, IMAGES_SIZE.width * 2, CV_8UC3);

		cv::Mat part = pair.colRange(0,  width);
		       cvtColor(imageLeft_l, part, cv::COLOR_GRAY2BGR);
		       part = pair.colRange( width,  width * 2);
		       cvtColor(imageRight_r, part, cv::COLOR_GRAY2BGR);
		       for (int j = 0; j <  height; j += 16){
		         cv::line(pair, cv::Point(0, j), cv::Point( width * 2, j),
		                  cv::Scalar(0, 255, 0));
		       }
		      
		cv::imshow("rectified", pair);
		cv::waitKey();
	}

	return P1;
}



void getDepthMap(const cv::Mat& imageLeft, const cv::Mat& imageRight, cv::Mat& depth_map, cv::Mat& Q){

 
	cv::Mat disp8 = getDisparity( imageLeft,  imageRight,  Q);
 	cv::reprojectImageTo3D(disp8, depth_map, Q);

}

cv::Mat getDisparity(const cv::Mat& imageLeft, const cv::Mat& imageRight, cv::Mat& Q){
	cv::Mat disp, disp8;
	sbm->compute(imageLeft, imageRight, disp);
	disp.convertTo( disp8, CV_32F, 1./16);
	return disp8;
}


cv::Mat getDisparityImage(const cv::Mat& imageLeft, const cv::Mat& imageRight, cv::Mat& Q){
	cv::Mat disp, disp8;
	sbm->compute(imageLeft, imageRight, disp);
	cv::normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
	return disp8;
}

 
