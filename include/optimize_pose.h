#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

#include <Eigen/Eigen>

#include <unsupported/Eigen/NonLinearOptimization>

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



cv::Mat toTransformationMat(const Eigen::VectorXf &x);

struct LMFunctor;

cv::Mat optimizeTrans(cv::Mat P, std::vector<cv::Point3f>& worldPointsPrev, 
	std::vector<cv::Point3f>& worldPointsCurr, std::vector<cv::Point2f>& currPoints,
	std::vector<cv::Point2f>& prevPoints);