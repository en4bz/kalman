#include "kalman.hpp"

kalman::kalman(ros::NodeHandle& nh, const cv::Mat& pmap, int spin_rate) : map(pmap.clone()), dt(1.0/(double)spin_rate) {

//  this->cmd_sub = nh.subscribe("odom", 1, &filter::propagate, this);
//	this->laser_sub = nh.subscribe("base_scan", 1, &filter::laser_update, this);
	this->bpgt_sub = nh.subscribe("base_pose_ground_truth", 1, &kalman::pose_callback, this);

	this->kf.transitionMatrix = cv::Mat::eye(3,3, CV_TYPE);
	this->kf.controlMatrix = cv::Mat::eye(3,3,CV_TYPE);
	this->kf.controlMatrix *= dt;
	this->kf.measurementMatrix = cv::Mat::eye(3,3,CV_TYPE);

    //Bound image by occupied cells.
    this->map.row(0) = cv::Scalar(0);
    this->map.row(map.size().width - 1) = cv::Scalar(0);
    this->map.col(0) = cv::Scalar(0);
    this->map.col(map.size().height - 1)  = cv::Scalar(0);
}

void kalman::predict(){
	return;
}

void kalman::correct(){
	return;
}

cv::Point2d kalman::toStage(cv::Point2i p) const{
    const double x_ratio = 50.0/map.size().width;
    const double y_ratio = 50.0/map.size().height;
    double x = p.x*x_ratio -25;
    double y = -(p.y - map.size().height)*y_ratio -25;
    return cv::Point2d(x,y);
}

cv::Point2i kalman::toImage(cv::Point2d p) const{
    const double x_ratio = map.size().width/50.0;
    const double y_ratio = map.size().height/50.0;
    double x = (p.x + 25)*x_ratio;
    double y = map.size().height - (p.y + 25 )*y_ratio;
    return cv::Point2i (x,y);
}

void kalman::pose_callback(const nav_msgs::Odometry msg){
    this->gt_y = -msg.pose.pose.position.y;
    this->gt_x = msg.pose.pose.position.x;

    double roll, pitch, heading;

    btQuaternion q (msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w);

    btMatrix3x3(q).getRPY(roll, pitch, heading);

    if(heading < -M_PI/2)
        heading += 5*M_PI/2;
    else
        heading += M_PI/2;

    this->gt_theta = heading;
}
