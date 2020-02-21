#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

#include <Eigen/Eigen>

#include <unsupported/Eigen/NonLinearOptimization>
#include "optimize_pose.h"

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
#include "optimize_pose.h"

struct LMFunctor
{
	

	cv::Mat P; //3x4 Projection Matrix
	std::vector<cv::Point3f> worldPointsPrev;
	std::vector<cv::Point3f> worldPointsCurr;
	std::vector<cv::Point2f> currPoints;
    std::vector<cv::Point2f> prevPoints;

	// Compute 'm' errors, one for each data point, for the given parameter values in 'x'
	int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
	{
		// 'x' has dimensions 6 x 1
		// It contains the 

		// 'fvec' has dimensions m x 1
		// It will contain the error for each data point.
		// std::cout << "operator" << std::endl;

		cv::Mat homoMatrix = toTransformationMat(x);


		for (int i = 0; i < worldPointsPrev.size(); i++) {

			cv::Point2f ptPrev = prevPoints.at(i);
			cv::Point2f ptCurr = currPoints.at(i);
			cv::Point3f ptWPrev = worldPointsPrev.at(i);
			cv::Point3f ptWCurr = worldPointsCurr.at(i);

			cv::Mat W_pt_prev = (cv::Mat_<double>(4,1) << ptWPrev.x, ptWPrev.y, ptWPrev.z, 1);
			cv::Mat W_pt_curr = (cv::Mat_<double>(4,1) << ptWCurr.x, ptWCurr.y, ptWCurr.z, 1);
			cv::Mat pt_prev = (cv::Mat_<double>(3,1) << ptPrev.x, ptPrev.y, 1);
			cv::Mat pt_curr = (cv::Mat_<double>(3,1) << ptCurr.x, ptCurr.y, 1);

			cv::Mat f1_repr = P*homoMatrix*W_pt_curr;
			f1_repr /= f1_repr.at<double>(2,0);



			cv::Mat f2_repr = P*(homoMatrix.inv())*W_pt_prev;
			f2_repr /= f2_repr.at<double>(2,0);

			

			cv::Mat e1 = f1_repr - pt_prev;
			cv::Mat e2 = f2_repr - pt_curr;
			 
			fvec(i) = e1.dot(e1)+e2.dot(e2);
			
		}
 

		return 0;
	}

	// Compute the jacobian of the errors
	int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
	{
		// 'x' has dimensions n x 1
		// It contains the current estimates for the parameters.

		// 'fjac' has dimensions m x n
		// It will contain the jacobian of the errors, calculated numerically in this case.
		
		float epsilon;
		epsilon = 1e-5f;

		for (int i = 0; i < x.size(); i++) {
			Eigen::VectorXf xPlus(x);
			xPlus(i) += epsilon;
			Eigen::VectorXf xMinus(x);
			xMinus(i) -= epsilon;

			Eigen::VectorXf fvecPlus(values());
			operator()(xPlus, fvecPlus);

			Eigen::VectorXf fvecMinus(values());
			operator()(xMinus, fvecMinus);

			Eigen::VectorXf fvecDiff(values());
			fvecDiff = (fvecPlus - fvecMinus) / (2.0f * epsilon);

			fjac.block(0, i, values(), 1) = fvecDiff;
		}

		return 0;
	}

	// Number of data points, i.e. values.
	int m;

	// Returns 'm', the number of values.
	int values() const { return m; }

	// The number of parameters, i.e. inputs.
	int n;

	// Returns 'n', the number of inputs.
	int inputs() const { return n; }

};



cv::Mat optimizeTrans(cv::Mat P, std::vector<cv::Point3f>& worldPointsPrev, 
	std::vector<cv::Point3f>& worldPointsCurr, std::vector<cv::Point2f>& currPoints,
	std::vector<cv::Point2f>& prevPoints){

	LMFunctor functor;
	functor.m = worldPointsCurr.size();
	functor.n = 6;
	functor.P = P; //3x4 Projection Matrix
	functor.worldPointsPrev = worldPointsPrev;
	functor.worldPointsCurr = worldPointsCurr;
	functor.currPoints = currPoints;
	functor.prevPoints = prevPoints;

	Eigen::VectorXf x(6);
	x(0) = 0.0;              
	x(1) = 0.0;             
	x(2) = 0.0; 
	x(3) = 0.0; 
	x(4) = 0.0; 
	x(5) = 0.0;   
	         

	Eigen::LevenbergMarquardt<LMFunctor, float> lm(functor);

	int status = lm.minimize(x);	
	std::cout << "LM optimization status: " << status << std::endl;
	std::cout << x << std::endl;

	cv::Mat transform = toTransformationMat(x);

	return transform;
	

}

cv::Mat toTransformationMat(const Eigen::VectorXf &x){

	const double roll = x(0); 
	const double pitch = x(1); 
	const double yaw = x(2);
	const double tx = x(3);
	const double ty = x(4);
	const double tz = x(5);


	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    Eigen::Matrix3d rotMatrix = q.matrix();

    cv::Mat homoMatrix = (cv::Mat_<double>(4,4) << rotMatrix(0,0), rotMatrix(0,1), rotMatrix(0,2), tx,
								    			   rotMatrix(1,0), rotMatrix(1,1), rotMatrix(1,2), ty,
								    			   rotMatrix(2,0), rotMatrix(2,1), rotMatrix(2,2), tz,
								    			   0,              0,              0,               1);



    return homoMatrix;
}



