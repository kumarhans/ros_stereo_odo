#include "opencv2/features2d/features2d.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>


#include "visualize.h"

 


void generateMarkerArray(visualization_msgs::MarkerArray& ma, std::vector<cv::Point3f>& points, cv::Mat& transform,
	int offset, int color){

	for (int i = 0; i< points.size(); i++){
		
		cv::Point3f pt = points.at(i);
		cv::Mat temp = (cv::Mat_<double>(4,1) << pt.x, pt.y, pt.z,1);
		cv::Mat ptTrans = transform*temp;

		visualization_msgs::Marker marker;

		marker.pose.position.x = ptTrans.at<double>(0,0);
        marker.pose.position.y = ptTrans.at<double>(1,0);
        marker.pose.position.z = ptTrans.at<double>(2,0);
        marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;


		marker.header.frame_id = "world";
		marker.ns = "ms";
	    marker.header.stamp = ros::Time();
	    marker.lifetime = ros::Duration(.5);
	    marker.id = i + offset;
	    marker.type = visualization_msgs::Marker::SPHERE;
	    marker.action = visualization_msgs::Marker::ADD;
	    marker.scale.x = 0.05;
	    marker.scale.y = 0.05;
	    marker.scale.z = 0.05;
	    marker.color.a = 1.0;  
	    marker.color.r = color;
	    marker.color.g = 1.0;
	    marker.color.b = 1.0;
	    ma.markers.push_back(marker);

	    
   
	}

	
};


geometry_msgs::TransformStamped getTf(cv::Mat& trans){
	geometry_msgs::TransformStamped transformStamped;

	double roll =  atan2( trans.at<double>(2,1),trans.at<double>(2,2) ) ;
	double pitch =  atan2( -trans.at<double>(2,0), 
	    std::pow( trans.at<double>(2,1)*trans.at<double>(2,1) +trans.at<double>(2,2)*trans.at<double>(2,2) ,0.5  )  ) ;
	double yaw =  atan2( trans.at<double>(1,0),trans.at<double>(0,0) ) ;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "pose";
	transformStamped.transform.translation.x = trans.at<double>(0,3);
	transformStamped.transform.translation.y = trans.at<double>(1,3);
	transformStamped.transform.translation.z = trans.at<double>(2,3);
	tf2::Quaternion q;
	
	q.setRPY(roll, pitch, yaw);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	return transformStamped;

};

void drawDebugImage(const cv::Mat& image, cv::Mat& debug_image, std::vector<cv::Point2f>& points, std::vector<cv::Point2f>& pointsPrev){
    
    image.copyTo(debug_image);
    // std::cout << "prevPoints" << pointsPrev.size() << std::endl;
    // std::cout << "currPoints" << points.size() << std::endl;

    for(int i=0;i<points.size();i++){
        if (pointsPrev.size() == points.size()){
            cv::circle(debug_image, points[i], 2, 1, -1, 8);
            //cv::circle(debug_image, pointsPrev[i], 1, 255, -1 ,8);
        }
        
    }
}