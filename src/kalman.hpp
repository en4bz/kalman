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
    ros::Subscriber cmd_sub;
    ros::Subscriber laser_sub;
    ros::Subscriber bpgt_sub;
public:
    kalman(ros::NodeHandle& nh, const cv::Mat& pmap);

    cv::Point2d toStage(cv::Point2i p) const;
    cv::Point2i toImage(cv::Point2d p) const;

    enum {OCCUPIED = 0, FREE = 255};
};

