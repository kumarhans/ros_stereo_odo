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

//https://github.com/opencv/opencv/blob/master/samples/cpp/lkdemo.cpp

 


cv::Mat rectifyImages(const cv::Mat& imageLeft, const cv::Mat& imageRight, cv::Mat& imageLeft_l, cv::Mat& imageRight_r, cv::Mat& Q, bool draw = false);
void getDepthMap(const cv::Mat& imageLeft, const cv::Mat& imageRight, cv::Mat& depth_map, cv::Mat& Q);
cv::Mat getDisparity(const cv::Mat& imageLeft, const cv::Mat& imageRight, cv::Mat& Q);
cv::Mat getDisparityImage(const cv::Mat& imageLeft, const cv::Mat& imageRight, cv::Mat& Q);


