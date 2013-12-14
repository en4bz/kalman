#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/random.hpp>
#include <boost/nondet_random.hpp>
#include <boost/random/normal_distribution.hpp>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

class kalman {
	const cv::Mat map;
	const double dt;
	double gt_x, gt_y, gt_theta;
	double linear, angular;
    ros::Subscriber cmd_sub;
    ros::Subscriber laser_sub;
    ros::Subscriber bpgt_sub;

    cv::Mat X;           //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
    cv::Mat F;   				//!< state transition matrix (F)
    cv::Mat controlMatrix;      //!< control matrix (B) (not used if there is no control)
    cv::Mat H;  				//!< measurement matrix (H)
    cv::Mat processNoiseCov;    //!< process noise covariance matrix (Q)
    cv::Mat measurementNoiseCov;//!< measurement noise covariance matrix (R)
    cv::Mat K;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
	cv::Mat P;


public:
    kalman(ros::NodeHandle& nh, const cv::Mat& pmap, int spin_rate);

	void pose_callback(const nav_msgs::Odometry msg);
	void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

	double ray_trace(const double x, const double y, const double theta, const double range, const double angle) const;

	void predict(const nav_msgs::Odometry msg);
	void correct();

    cv::Point2d toStage(cv::Point2i p) const;
    cv::Point2i toImage(cv::Point2d p) const;

    enum {OCCUPIED = 0, FREE = 255};
	const static int CV_TYPE = CV_64F;
};

