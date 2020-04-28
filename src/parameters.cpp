#include "parameters.h"


double fx;
double fy;
double cx;
double cy;
double k1;
double k2;
double p1;
double p2;
double k3;
int width;
int height;
double baseline;

cv::Mat cameraMatrix1;
cv::Mat cameraMatrix2;
cv::Mat distCoeffs1;
cv::Mat distCoeffs2;


cv::Mat STEREO_R;
cv::Mat STEREO_T;
std::vector<cv::Mat> CM; // Stores 3x3 Camera Matricies
std::vector<cv::Mat> DC; // Stores 5x1 Camera Matricies
cv::Size IMAGES_SIZE;
std::string IMAGE_L_TOPIC;
std::string IMAGE_R_TOPIC;



template <typename T>
void readParametersHelper(ros::NodeHandle &nh, std::string name, T &ans){
    if (nh.getParam(name, ans)){
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else{
        ROS_ERROR_STREAM("Failed to load " << name);
        nh.shutdown();
    }
  
}

void readParameters(ros::NodeHandle &nh){
 
    readParametersHelper<double>(nh,"/fx",fx);
    readParametersHelper<double>(nh,"/fy",fy);
    readParametersHelper<double>(nh,"/cx",cx);
    readParametersHelper<double>(nh,"/cy",cy);
    readParametersHelper<double>(nh,"/k1",k1);
    readParametersHelper<double>(nh,"/k2",k2);
    readParametersHelper<double>(nh,"/p1",p1);
    readParametersHelper<double>(nh,"/p2",p2);
    readParametersHelper<double>(nh,"/k3",k3);
    readParametersHelper<double>(nh,"/baseline",baseline);

    readParametersHelper<int>(nh,"/width",width);
    readParametersHelper<int>(nh,"/height",height);

    readParametersHelper<std::string>(nh,"/image0_topic",IMAGE_L_TOPIC);
    readParametersHelper<std::string>(nh,"/image1_topic",IMAGE_R_TOPIC);

    

    cameraMatrix1 = (cv::Mat_<double>(3,3) << fx, 0, cx, 
	                                          0, fy, cy, 
	                                          0, 0, 1);

    cameraMatrix2 = (cv::Mat_<double>(3,3) << fx, 0, cx, 
	                                          0, fy, cy, 
	                                          0, 0, 1);


    distCoeffs1 = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);
    distCoeffs2 = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);


    STEREO_R = (cv::Mat_<double>(3,3) << 1, 0, 0, 
	                                  0, 1, 0, 
	                                  0, 0, 1);
    STEREO_T = (cv::Mat_<double>(3,1) << 0, -baseline, 0);


    CM.push_back(cameraMatrix1);
	CM.push_back(cameraMatrix2);
	DC.push_back(distCoeffs1);
	DC.push_back(distCoeffs2);

    IMAGES_SIZE = cv::Size_<int>(width,height);


}






