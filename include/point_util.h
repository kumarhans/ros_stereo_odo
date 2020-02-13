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



// void getWorldPoints(const std::vector<cv::Point2f>& Points,	std::vector<cv::Point3f>& worldPoints, const cv::Mat& disparity,  cv::Mat& Q);

std::vector<int> initializeClique(std::vector<std::vector<int>> ad_mat);

int op_isSame(int i,int j);

double getDistance(cv::Point3f& pt1, cv::Point3f& pt2);

std::vector<std::vector<int>> getAdjacenyMatrix(std::vector<cv::Point3f>& currPoints, 
    std::vector<cv::Point3f>& prevPoints, double thresh);

void getWorldPoints(const std::vector<cv::Point2f>& PointsPrev,	const std::vector<cv::Point2f>& PointsCurr, 
    std::vector<cv::Point3f>& worldPointsPrev, std::vector<cv::Point3f>& worldPointsCurr, 
    const cv::Mat& disparityPrev, const cv::Mat& disparityCurr, cv::Mat& Q);

void updateClique(std::vector<int> potentialSet, std::vector<int>& clique, 
    const std::vector<std::vector<int>>& ad_mat);


std::vector<int> potentialNodes(const std::vector<int>& clique, const std::vector<std::vector<int>>& ad_mat);

       