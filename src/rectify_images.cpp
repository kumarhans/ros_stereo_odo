
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
 


double fx= 554.3826904296875;
double fy= 554.3826904296875;
double cx= 320.0;
double cy= 240.0;
double k1= 0.0;
double k2= 0.0;
double p1= 0.0;
double p2= 0.0;
double k3= 0.0;
int width= 480;
int height= 360;

// Camera.k1: 0.0
// Camera.k2: 0.0
// Camera.p1: 0.0
// Camera.p2: 0.0
// Camera.k3: 0.0

// Camera.width: 480
// Camera.height: 360

//stereoRectify
//initUndistortRectifyMap
//remap

std::vector<cv::Mat> CM; // Stores 3x3 Camera Matricies
cv::Mat cameraMatrix1 = (cv::Mat_<double>(3,3) << fx, 0, cx, 
	                                          0, fy, cy, 
	                                          0, 0, 1);

cv::Mat cameraMatrix2 = (cv::Mat_<double>(3,3) << fx, 0, cx, 
	                                          0, fy, cy, 
	                                          0, 0, 1);




std::vector<cv::Mat> DC; // Stores 5x1 Camera Matricies
cv::Mat distCoeffs1 = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);
cv::Mat distCoeffs2 = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);



cv::Size imageSize = cv::Size_<int>(width,height);


cv::Mat R = (cv::Mat_<double>(3,3) << 1, 0, 0, 
	                                  0, 1, 0, 
	                                  0, 0, 1);

cv::Mat T = (cv::Mat_<double>(3,1) << 0, 0.07, 0);




 


 
cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,16,3);





void rectifyImages(const cv::Mat& imageLeft, const cv::Mat& imageRight, cv::Mat& imageLeft_l, cv::Mat& imageRight_r, cv::Mat& Q){

	CM.push_back(cameraMatrix1);
	CM.push_back(cameraMatrix2);

	DC.push_back(distCoeffs1);
	DC.push_back(distCoeffs2);

	cv::Mat R1, R2, P1, P2;
	cv::Mat rmap[2][2];

	cv::stereoRectify(CM[0], DC[0],
                  CM[1], DC[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  cv::CALIB_ZERO_DISPARITY, 1, imageSize);

	cv::initUndistortRectifyMap(CM[0], DC[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	cv::initUndistortRectifyMap(CM[1], DC[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	cv::remap(imageLeft, imageLeft_l, rmap[0][0], rmap[1][1], cv::INTER_LINEAR);
	cv::remap(imageRight, imageRight_r, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);

}



void getDepthMap(const cv::Mat& imageLeft, const cv::Mat& imageRight, cv::Mat& depth_map, cv::Mat& Q){

 
	cv::Mat disp8 = getDisparity( imageLeft,  imageRight,  Q);
 	cv::reprojectImageTo3D(disp8, depth_map, Q);

}

cv::Mat getDisparity(const cv::Mat& imageLeft, const cv::Mat& imageRight, cv::Mat& Q){


    // int numberOfDisparities = (( width/8) + 15) & -16;

    // sgbm->setPreFilterCap(63);
    // int sgbmWinSize =  3;
    // sgbm->setBlockSize(sgbmWinSize);

    // int cn = imageLeft.channels();

    // sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    // sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    // sgbm->setMinDisparity(0);
    // sgbm->setNumDisparities(numberOfDisparities);
    // sgbm->setUniquenessRatio(10);
    // sgbm->setSpeckleWindowSize(100);
    // sgbm->setSpeckleRange(32);
    // sgbm->setDisp12MaxDiff(1);


	cv::Mat disp, disp8;
	sgbm->compute(imageLeft, imageRight, disp);

	

	//0/0;

	disp.convertTo( disp8, CV_32F, 1./16);

	//std::cout << disp8<< std::endl;
	 
	//cv::normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

	return disp8;

}

 
void getWorldPoints(const std::vector<cv::Point2f>& Points,	std::vector<cv::Point3f>& worldPoints, const cv::Mat& disparity,  cv::Mat& Q){

	cv::Mat pt_mat;
	cv::Mat w_pt_mat;
	//Q.at<double>(2,3)= -1*(Q.at<double>(2,3));

	 for (int i = 0; i<Points.size(); i++){
        cv::Point2f pt = Points.at(i);
        //std::cout << disparity  << std::endl;
        // std::cout << (int)pt.x << std::endl;
         
        double d = disparity.at<float>((int)pt.y,(int)pt.x);

        
        if (d > .1){
        	pt_mat = (cv::Mat_<double>(4,1) << (int)pt.x, (int)pt.y, d, 1);
        	w_pt_mat = Q*pt_mat;
        	w_pt_mat /= w_pt_mat.at<double>(3,0);
        	
        	std::cout << d <<  std::endl;
         
        	std::cout << pt_mat<< std::endl;
        	std::cout << w_pt_mat << std::endl;
        	cv::Point3f w_pt(w_pt_mat.at<double>(0,0),w_pt_mat.at<double>(1,0),w_pt_mat.at<double>(2,0));

        	if (w_pt.x < 3.5 && w_pt.y < 3.5 && w_pt.z < 3.5){
        		worldPoints.push_back(w_pt);

        	} 
        	
        }
        
    }


}