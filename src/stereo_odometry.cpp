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

    R_init = (cv::Mat_<double>(3,3) << 1, 0, 0, 
                                       0, cos(M_PI /3), -sin(M_PI/3), 
                                       0, sin(M_PI /3), cos(M_PI/3));
    T_init = (cv::Mat_<double>(3,1) << 0, 0, .75);

    // Q = (cv::Mat_<double>(4,4) << 1, 0, 0, -320, 
    //                                       0, 1, 0, -240, 
    //                                       0, 0, 0, -554.3826904296875,
    //                                       0, 0, -14.28571428571428, 0);


}

void StereoOdometry::imageCallback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg){
    try {
        

        curr_image_left = cv_bridge::toCvShare(left_msg, "mono8")->image;
        curr_image_right = cv_bridge::toCvShare(right_msg, "mono8")->image;

        rectifyImages( curr_image_left,  curr_image_right, curr_image_left, curr_image_right,Q);


        //getDepthMap( curr_image_left,  curr_image_right, depth_map_curr, Q);
        //getDepthMap( prev_image_left,  prev_image_right, depth_map_prev, Q); 
        cv::Mat disp = getDisparity(curr_image_left, curr_image_right,  Q);


        
        if (!init) {
            initKeypoints(curr_image_left, keypoints, currPoints);
            init = true;
        } else {
            matchFeatures(prev_image_left, curr_image_left, prevPoints, currPoints);

            cv::Mat dispPrev = getDisparity(prev_image_left, prev_image_right,  Q);
            cv::Mat dispCurr = getDisparity(curr_image_left, curr_image_right,  Q);
            std::vector<cv::Point3f> currWorldPoints;
            std::vector<cv::Point3f> prevWorldPoints;

            getWorldPoints(prevPoints, currPoints, prevWorldPoints, currWorldPoints, dispPrev, dispCurr, Q);


            // std::cout << prevPoints.size() << std::endl;
            // std::cout << currPoints.size() << std::endl;

            // std::cout << prevWorldPoints.size() << std::endl;
            // std::cout << currWorldPoints.size() << std::endl;

            std::vector<std::vector<int>> ad_mat = getAdjacenyMatrix(prevWorldPoints, currWorldPoints, .1);

            std::vector<int> clique = initializeClique(ad_mat);
            // for (int i = 0; i < clique.size(); i++) { 
            //     std::cout<< clique[i] << std::endl;
            // } 


            std::vector<int> potSet = potentialNodes(clique, ad_mat);

            while (std::accumulate(potSet.begin(), potSet.end(), 0) > 0){
                updateClique(potSet ,clique,ad_mat);
                potSet = potentialNodes(clique, ad_mat);


            }

            //std::cout << clique.size()<< std::endl;
            // for (int i = 0; i < clique.size(); i++) { 
            //     std::cout<< clique[i] << std::endl;
            // } 


            
            
            

            // for (int i = 0; i < ad_mat.size(); i++) { 
            //     for (int j = 0; j < ad_mat.size(); j++){ 
            //         std::cout<< ad_mat[i][j]<< " "; 
            //     } 
            //     std::cout<< "\n"; 
            // } 


            

            


            for (int i = 0; i<currWorldPoints.size(); i++){

               

                if (std::find(clique.begin(), clique.end(), i) != clique.end()){
                    marker.color.g = 1.0;
                }
                else{
                    marker.color.g = 0.0;
                }


                cv::Point3f pt = currWorldPoints.at(i);
                cv::Mat temp = (cv::Mat_<double>(3,1) << pt.x, pt.y, pt.z);
                cv::Mat Final = R_init*temp+T_init;


                marker.pose.position.x = Final.at<double>(0,0);
                marker.pose.position.y = Final.at<double>(1,0);
                marker.pose.position.z = Final.at<double>(2,0);
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
        
 
