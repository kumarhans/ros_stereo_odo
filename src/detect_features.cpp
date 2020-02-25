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
#include "detect_features.h"

//https://github.com/opencv/opencv/blob/master/samples/cpp/lkdemo.cpp



void initKeypoints(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& points){
    cv::FAST(image, keypoints, 10, true);
    cv::KeyPoint::convert(keypoints, points);
}

void matchFeatures(const cv::Mat& prevImage, const cv::Mat& currImage, std::vector<cv::Point2f>& prevPoints, 
    std::vector<cv::Point2f>& currPoints){

    assert(!prevPoints.empty());
    std::vector<float> err;               
    std::vector<uchar> status;     
    cv::Size winSize=cv::Size(21,21);   

    cv::TermCriteria termcrit=cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
    calcOpticalFlowPyrLK(prevImage, currImage, prevPoints, currPoints, status, err, winSize, 3, termcrit, 0, 0.001);
    deleteFeatures(prevPoints, currPoints, status);

    //Print Size of Feature Vectors
    // std::cout << "prevPoints" << prevPoints.size() << std::endl;
    // std::cout << "currPoints" << currPoints.size() << std::endl;
}

void deleteFeatures(std::vector<cv::Point2f>& prevPoints, std::vector<cv::Point2f>& currPoints, std::vector<uchar> status){
    int offset = 0;
   
    for (int i = 0;i<status.size(); i++){
        cv::Point2f pt = currPoints.at(i- offset);
        if (status[i] == 0 || pt.x < 0 || pt.y < 0){
            currPoints.erase(currPoints.begin() + (i - offset));
            prevPoints.erase(prevPoints.begin() + (i - offset));
            offset ++;
        }
    }
   
}







