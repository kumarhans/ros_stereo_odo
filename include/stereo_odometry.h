#ifndef STER_h
#define STER_h

//Some Ros Packages
#include <ros/package.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "opencv2/features2d/features2d.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
 

 
class StereoOdometry
{
public:
    
    StereoOdometry(ros::NodeHandle &nodehandle,image_transport::ImageTransport &imagehandle);
    ~StereoOdometry();

    bool init;
    bool depthMap;
    bool visualize;

    cv::Mat prev_image_left;
    cv::Mat prev_image_right;

    cv::Mat curr_image_left;
    cv::Mat curr_image_right;
    
    cv::Mat debug_image;

    std::vector<cv::Point2f> currPoints;
    std::vector<cv::Point2f> prevPoints;
    std::vector<cv::KeyPoint> keypoints;

    cv::Mat disparity;
    cv::Mat Q;
    cv::Mat depth_map_prev;
    cv::Mat depth_map_curr;

    cv::Mat H_init;
    cv::Mat pose;

    std::vector<std::vector<int>> ad_mat;
  
    
private:

    void initializeSubsAndPubs();
    void getInitialRot(double angleDown, double height);
    void visualizePoints(std::vector<cv::Point3f>& currWorldPoints, std::vector<cv::Point3f>& prevWorldPoints);

    ros::NodeHandle nh;
    image_transport::ImageTransport it;

    image_transport::SubscriberFilter left_sub;
    image_transport::SubscriberFilter right_sub;

    image_transport::Publisher debug_pub;
    ros::Publisher pose_pub;
    
    ros::Publisher vis_pub;


    void imageCallback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> *sync;
     
};


#endif