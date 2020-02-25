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
 
void generateMarkerArray(visualization_msgs::MarkerArray& ma, std::vector<cv::Point3f>& points,
	cv::Mat& transform, int offset, int color = 1);

geometry_msgs::TransformStamped getTf(cv::Mat& trans);

void drawDebugImage(const cv::Mat& image, cv::Mat& debug_image, std::vector<cv::Point2f>& points, std::vector<cv::Point2f>& pointsPrev);