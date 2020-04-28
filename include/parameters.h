#ifndef PARAM_h
#define PARAM_h

#include <ros/ros.h>
#include <vector>
 
#include <opencv2/opencv.hpp>
 
 
extern double fx;
extern double fy;
extern double cx;
extern double cy;
extern double k1;
extern double k2;
extern double p1;
extern double p2;
extern double k3;
extern int width;
extern int height;

extern cv::Mat cameraMatrix1;
extern cv::Mat cameraMatrix2;

extern cv::Mat distCoeffs1;
extern cv::Mat distCoeffs2;

extern cv::Mat STEREO_R;
extern cv::Mat STEREO_T;
extern std::vector<cv::Mat> CM; // Stores 3x3 Camera Matricies
extern std::vector<cv::Mat> DC; // Stores 5x1 Camera Matricies
extern cv::Size IMAGES_SIZE;
extern std::string IMAGE_L_TOPIC;
extern std::string IMAGE_R_TOPIC;

template <typename T>
void readParametersHelper(ros::NodeHandle &nh, std::string name, T &ans);

void readParameters(ros::NodeHandle &nh);

 

#endif