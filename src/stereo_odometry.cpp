//Some Ros Packages
#include <ros/package.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <numeric>
 
#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif
 
//Standard utilities
#include <iostream>

//Header Files
#include "stereo_odometry.h"
#include "detect_features.h"
#include "rectify_images.h"
#include "point_util.h"
#include "optimize_pose.h"
#include "visualize.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>

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

void StereoOdometry::getInitialRot(double angleDown, double height){

    cv::Mat Yby90 = (cv::Mat_<double>(3,3) <<   cos(M_PI/2), 0.0, sin(M_PI/2), 
                                                0.0,         1.0,          0.0, 
                                                -sin(M_PI/2), 0.0, cos(M_PI/2));

    cv::Mat ZbyNeg90 = (cv::Mat_<double>(3,3) <<  cos(M_PI /2), sin(M_PI/2), 0.0, 
                                                  -sin(M_PI /2), cos(M_PI/2),  0.0, 
                                                            0.0,          0.0,  1.0);


    cv::Mat PitchDownNegX = (cv::Mat_<double>(3,3) <<    1,             0,             0, 
                                                         0, cos(M_PI/6) , sin(M_PI/6), 
                                                         0, -sin(M_PI /6),  cos(M_PI/6));

    cv::Mat R_init = Yby90*ZbyNeg90*PitchDownNegX;

    cv::Mat T_init = (cv::Mat_<double>(3,1) << 0, 0, height);
    cv::Mat homoRow = (cv::Mat_<double>(1,4) << 0, 0, 0,1);
    cv::Mat temp;

    cv::hconcat(R_init,T_init,temp);
    cv::vconcat(temp,homoRow,H_init);

}

void StereoOdometry::initializeSubsAndPubs(){

    ROS_INFO("Initializing Subscribers and Publishers");
    left_sub.subscribe(it,"/left_r200/camera/color/image_raw", 1);
    right_sub.subscribe(it,"/right_r200/camera/color/image_raw", 1);
     
    sync = new message_filters::Synchronizer<MySyncPolicy> (MySyncPolicy(10), left_sub, right_sub);
    sync -> registerCallback(boost::bind(&StereoOdometry::imageCallback, this, _1, _2 ));   

    debug_pub = it.advertise("/ros_stereo_odo/debug_image", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ros_stereo_odo/pose", 100);
    vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );

    double angle = M_PI/6;
    double height = .75;
    getInitialRot(angle, height);
    depthMap = false;
    visualize = true;

}

void StereoOdometry::visualizePoints(std::vector<cv::Point3f>& currWorldPoints, std::vector<cv::Point3f>& prevWorldPoints){
    visualization_msgs::MarkerArray ma;
    generateMarkerArray(ma, currWorldPoints, H_init, 0);
    generateMarkerArray(ma, prevWorldPoints, H_init, currWorldPoints.size(), .5);
    vis_pub.publish(ma.markers);

    drawDebugImage(curr_image_left,  debug_image, currPoints, prevPoints);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", debug_image).toImageMsg();
    debug_pub.publish(msg);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped = getTf(H_init);
    br.sendTransform(transformStamped);
}



void StereoOdometry::imageCallback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg){
    try {
        
        //Get Images, Rectify, and Calculate Disparity
        curr_image_left = cv_bridge::toCvShare(left_msg, "mono8")->image;
        curr_image_right = cv_bridge::toCvShare(right_msg, "mono8")->image;
        cv::Mat P = rectifyImages( curr_image_left,  curr_image_right, curr_image_left, curr_image_right,Q);
        cv::Mat disp = getDisparity(curr_image_left, curr_image_right,  Q);


        //Optional Get Depth Maps
        if (depthMap){
            getDepthMap( curr_image_left,  curr_image_right, depth_map_curr, Q);
            getDepthMap( prev_image_left,  prev_image_right, depth_map_prev, Q); 
        }
        
        

        if (!init) {
            //Initialize KeyPoints Features
            initKeypoints(curr_image_left, keypoints, currPoints);
            init = true;

        } else {

            //Match Features and Calculate Disparity
            matchFeatures(prev_image_left, curr_image_left, prevPoints, currPoints);
            cv::Mat dispPrev = getDisparity(prev_image_left, prev_image_right,  Q);
            cv::Mat dispCurr = getDisparity(curr_image_left, curr_image_right,  Q);
            
            //Project Points to 3D
            std::vector<cv::Point3f> currWorldPoints;
            std::vector<cv::Point3f> prevWorldPoints;
            getWorldPoints(prevPoints, currPoints, prevWorldPoints, currWorldPoints, dispPrev, dispCurr, Q);
            

            //Prune Cloud to get Inliers
            updateCloud(prevPoints, currPoints, prevWorldPoints, currWorldPoints);
            

            //Optimize to get pose change and update pose
            cv::Mat optoTrans = optimizeTrans( P,  prevWorldPoints,  currWorldPoints,currPoints,prevPoints);
            H_init = H_init*optoTrans;


            //Optionally Visualize Results
            if (visualize){
                visualizePoints(currWorldPoints, prevWorldPoints);
            }   

        }

        //Update Prev
        prevPoints = currPoints;
        std::vector<cv::Point2f> tmp;
        currPoints = tmp;
        prev_image_left = curr_image_left;
        prev_image_right = curr_image_right;

 
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
        
 
