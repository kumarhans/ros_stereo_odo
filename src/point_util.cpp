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
#include <cmath>
#include <numeric>

#include "point_util.h"







void getWorldPoints(const std::vector<cv::Point2f>& PointsPrev,	const std::vector<cv::Point2f>& PointsCurr, 
    std::vector<cv::Point3f>& worldPointsPrev, std::vector<cv::Point3f>& worldPointsCurr, 
    const cv::Mat& disparityPrev, const cv::Mat& disparityCurr, cv::Mat& Q){

	cv::Mat pt_mat_curr;
	cv::Mat w_pt_mat_curr;
    cv::Mat pt_mat_prev;
    cv::Mat w_pt_mat_prev;

	 for (int i = 0; i<PointsPrev.size(); i++){

        cv::Point2f ptPrev = PointsPrev.at(i);
        double dPrev = disparityPrev.at<float>((int)ptPrev.y,(int)ptPrev.x);

        cv::Point2f ptCurr = PointsCurr.at(i);
        double dCurr = disparityCurr.at<float>((int)ptCurr.y,(int)ptCurr.x);

        if (dPrev > .01 && dCurr > .01){
        	pt_mat_prev = (cv::Mat_<double>(4,1) << (int)ptPrev.x, (int)ptPrev.y, dPrev, 1);
        	w_pt_mat_prev = Q*pt_mat_prev;
        	w_pt_mat_prev /= w_pt_mat_prev.at<double>(3,0);
        	cv::Point3f w_pt_prev(w_pt_mat_prev.at<double>(0,0),w_pt_mat_prev.at<double>(1,0),w_pt_mat_prev.at<double>(2,0));
        	worldPointsPrev.push_back(w_pt_prev);

            pt_mat_curr = (cv::Mat_<double>(4,1) << (int)ptCurr.x, (int)ptCurr.y, dCurr, 1);
            w_pt_mat_curr = Q*pt_mat_curr;
            w_pt_mat_curr /= w_pt_mat_curr.at<double>(3,0);
            cv::Point3f w_pt_curr(w_pt_mat_curr.at<double>(0,0),w_pt_mat_curr.at<double>(1,0),w_pt_mat_curr.at<double>(2,0));
            worldPointsCurr.push_back(w_pt_curr);


        }   
    }
}


double getDistance(cv::Point3f& pt1, cv::Point3f& pt2){
    double distance = sqrt(pow(pt1.x-pt2.x,2) + pow(pt1.y-pt2.y,2) + pow(pt1.z-pt2.z,2));
    return distance;
}



std::vector<std::vector<int>> getAdjacenyMatrix(std::vector<cv::Point3f>& currPoints, 
    std::vector<cv::Point3f>& prevPoints, double thresh){

    std::vector<std::vector<int>> ad_mat(currPoints.size() , std::vector<int> (currPoints.size(),0));
    int pointLen = currPoints.size();

    for (int i = 0;i<pointLen; i++){
        for (int j = 0;j<pointLen; j++){
            double distCurr = getDistance(currPoints.at(i),currPoints.at(j));
            double distPrev = getDistance(prevPoints.at(i),prevPoints.at(j));
            if (abs(distCurr-distPrev) < thresh){
                ad_mat[i][j] = 1;
            }
        }
    }
    return ad_mat;
}
 

std::vector<int> initializeClique(std::vector<std::vector<int>> ad_mat){

    
    int node_max;
    int edge_max = 0; 

    //Find the biggest set in ad matrix
    for(int i = 0; i < ad_mat.size(); i ++){
        int connections = std::accumulate(ad_mat[i].begin(), ad_mat[i].end(), 0);
        if (connections > edge_max){
            edge_max = connections;
            node_max = i;
        }
    }

    std::vector<int> clique;
    clique.push_back(node_max);

    return clique;
}


int op_isSame(int i,int j) { return (i==1 && j == 1) ? 1 : 0;}


void updateClique(std::vector<int> potentialSet, std::vector<int>& clique, 
    const std::vector<std::vector<int>>& ad_mat){
    int maxMatches = 0;
    int currMax;
    int numMatches;

    for (int i=0; i < potentialSet.size();i++){
        if (potentialSet[i] == 1){
            numMatches = 0;
            for (int j=0; j < potentialSet.size();j++){
                if (potentialSet[j] == 1 && ad_mat[i][j] == 1){
                    numMatches ++;
                }
            }
            if (numMatches > maxMatches){
                currMax = i;
                maxMatches = numMatches;
            }
        }
    }

    if (maxMatches > 0){
        clique.push_back(currMax);
    }
}


std::vector<int> potentialNodes(const std::vector<int>& clique, const std::vector<std::vector<int>>& ad_mat){

    std::vector<int> potentialSet =  ad_mat[clique[0]];

    if (clique.size() > 1){
        for (int i=1; i < clique.size();i++){
            std::transform(potentialSet.begin(), potentialSet.end(), ad_mat[clique[i]].begin(), potentialSet.begin(), op_isSame);
        }
    }

    for (int i=0; i < clique.size();i++){
        potentialSet[clique[i]] = 0;
    }

    return potentialSet;

}

            