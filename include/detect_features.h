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



void initKeypoints(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& points);

void matchFeatures(const cv::Mat& prevImage, const cv::Mat& currImage, std::vector<cv::Point2f>& prevPoints, 
    std::vector<cv::Point2f>& currPoints);

void deleteFeatures(std::vector<cv::Point2f>& prevPoints, std::vector<cv::Point2f>& currPoints, std::vector<uchar> status);