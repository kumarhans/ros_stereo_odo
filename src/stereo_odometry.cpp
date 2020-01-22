//Some Ros Packages
#include <ros/package.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
 
 
//Standard utilities
#include <iostream>

//Header Files
#include "stereo_odometry.h"
#include "detect_features.h"
 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_stereo_odo");
    ros::NodeHandle nh; 
    image_transport::ImageTransport it(nh);

    StereoOdometry node(nh, it);
    ros::Rate loop_rate(100);
    ros::spin();

    return 0;
}

StereoOdometry::StereoOdometry(ros::NodeHandle &nodehandle,image_transport::ImageTransport &imagehandle):nh(nodehandle),it(imagehandle){
    initializeSubsAndPubs();
    init = false;
}

void StereoOdometry::initializeSubsAndPubs(){

    ROS_INFO("Initializing Subscribers and Publishers");
    left_sub.subscribe(it,"/left_r200/camera/color/image_raw", 1);
    right_sub.subscribe(it,"/left_r200/camera/color/image_raw", 1);
     
    sync = new message_filters::Synchronizer<MySyncPolicy> (MySyncPolicy(10), left_sub, right_sub);
    sync -> registerCallback(boost::bind(&StereoOdometry::imageCallback, this, _1, _2 ));   

    debug_pub = it.advertise("/ros_stereo_odo/debug_image", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ros_stereo_odo/pose", 100);


}

void StereoOdometry::imageCallback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg){
    try {
        
        curr_image_left = cv_bridge::toCvShare(left_msg, "mono8")->image;
        curr_image_right = cv_bridge::toCvShare(right_msg, "mono8")->image;
        

        if (!init) {
            initKeypoints(curr_image_left, keypoints, currPoints);
            init = true;
        } else {
            //initKeypoints(curr_image_left, keypoints, currPoints);
            matchFeatures(prev_image_left, curr_image_left, prevPoints, currPoints);
        }
        //std::cout << currPoints.size() << std::endl;
        drawFeatures(curr_image_left,  debug_image, currPoints);
        
        prevPoints = currPoints;
        std::vector<cv::Point2f> tmp;
        currPoints = tmp;
        prev_image_left = curr_image_left;
        prev_image_right = curr_image_right;
        

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", debug_image).toImageMsg();
        debug_pub.publish(msg);
        std::cout << "found Image" << std::endl;

        if (prevPoints.size() < 100){
            init = false;
        }

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

StereoOdometry::~StereoOdometry () {
    delete sync;
}
        
 
