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
#include "rectify_images.h"

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
    right_sub.subscribe(it,"/right_r200/camera/color/image_raw", 1);
     
    sync = new message_filters::Synchronizer<MySyncPolicy> (MySyncPolicy(10), left_sub, right_sub);
    sync -> registerCallback(boost::bind(&StereoOdometry::imageCallback, this, _1, _2 ));   

    debug_pub = it.advertise("/ros_stereo_odo/debug_image", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ros_stereo_odo/pose", 100);

    vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.lifetime = ros::Duration(1.5);
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    Q = (cv::Mat_<double>(4,4) << 1, 0, 0, -320, 
                                          0, 1, 0, -240, 
                                          0, 0, 0, -554.3826904296875,
                                          0, 0, -14.28571428571428, 0);


}

void StereoOdometry::imageCallback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg){
    try {
        

        curr_image_left = cv_bridge::toCvShare(left_msg, "mono8")->image;
        curr_image_right = cv_bridge::toCvShare(right_msg, "mono8")->image;

        //rectifyImages( curr_image_left,  curr_image_right, curr_image_left, curr_image_right,Q);\



        //getDepthMap( curr_image_left,  curr_image_right, depth_map_curr, Q);
        //getDepthMap( prev_image_left,  prev_image_right, depth_map_prev, Q); 


        
        if (!init) {
            initKeypoints(curr_image_left, keypoints, currPoints);
            init = true;
        } else {
            matchFeatures(prev_image_left, curr_image_left, prevPoints, currPoints);

            cv::Mat dispPrev = getDisparity(prev_image_left, prev_image_right,  Q);
            cv::Mat dispCurr = getDisparity(curr_image_left, curr_image_right,  Q);
            std::vector<cv::Point3f> currWorldPoints;
            std::vector<cv::Point3f> prevWorldPoints;
            getWorldPoints(prevPoints, prevWorldPoints, dispPrev,Q);
            getWorldPoints(currPoints, currWorldPoints, dispCurr,Q);


            


            for (int i = 0; i<currWorldPoints.size(); i++){
                cv::Point3f pt = currWorldPoints.at(i);

                marker.pose.position.x = pt.x;
                marker.pose.position.y = pt.y;
                marker.pose.position.z = pt.z;
                marker.id = i;
                vis_pub.publish( marker );
            
            }

        }


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
        
 
