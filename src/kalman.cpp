#include "kalman.hpp"

kalman::kalman(ros::NodeHandle& nh, const cv::Mat& pmap, double x_init, double y_init, double theta_init, int spin_rate)
	: map(pmap.clone()), dt(1.0/(double)spin_rate) {

	this->cmd_sub = nh.subscribe("odom", 1, &kalman::predict, this);
	this->laser_sub = nh.subscribe("base_scan", 1, &kalman::laser_callback, this);
	this->bpgt_sub = nh.subscribe("base_pose_ground_truth", 1, &kalman::pose_callback, this);

	this->X[0] = x_init;
	this->X[1] = y_init;
	this->X[2] = theta_init;

	this->F(0,0) = 1;
	this->F(1,1) = 1;
	this->F(2,2) = 1;

	this->I(0,0) = 1;
	this->I(1,1) = 1;
	this->I(2,2) = 1;

    //Bound image by occupied cells.
    this->map.row(0) = cv::Scalar(0);
    this->map.row(map.size().width - 1) = cv::Scalar(0);
    this->map.col(0) = cv::Scalar(0);
    this->map.col(map.size().height - 1)  = cv::Scalar(0);
}


void kalman::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
 	const int size = (msg->angle_max - msg->angle_min) / msg->angle_increment;

	laser.clear();

	laser.push_back( rangle(msg->ranges[0], msg->angle_min));
	laser.push_back( rangle(msg->ranges[size-1], msg->angle_max));
	laser.push_back( rangle(msg->ranges[size/2], 0));

	return;
}

void kalman::predict(const nav_msgs::Odometry msg){
	this->X[0] += linear * dt * cos( X[2] );
	this->X[1] += linear * dt * sin( X[2] );
	this->X[2] += angular * dt;

	this->F(0,2) = -linear * dt * sin( X[2] ); //t+1 ?
	this->F(1,2) =  linear * dt * cos( X[2] ); //t+1 ?

	P = F * P * F.t(); // + Q

	this->linear = msg.twist.twist.linear.x;
	this->angular = msg.twist.twist.angular.z;
	return;
}

void kalman::correct(){
	const double x = this->X[0];
	const double y = this->X[1];
	const double theta = this->X[2];
	const double dx = 0.1;
	const double dy = 0.1;
	const double dtheta = 0.1;

	cv::Vec3d res;

	for(size_t i = 0; i < laser.size(); i++){
		const double angle = laser[i].angle;
		res[i] = laser[i].range - ray_trace(x,y,theta,5, angle);
		//These forumals are wrong! They're backward f'(x) = f(x+h) - f(x-h) / 2h
		this->H(i,0) = (ray_trace(x-dx,y,theta,5,angle) - ray_trace(x+dx,y,theta,5,angle) ) / 2*dx;
		this->H(i,1) = (ray_trace(x,y-dy,theta,5,angle) - ray_trace(x,y+dy,theta,5,angle) ) / 2*dy;
		this->H(i,2) = (ray_trace(x,y,theta-dtheta,5,angle) - ray_trace(x,y,theta+dtheta,5,angle) ) / 2*dtheta;
	}
	std::cout << cv::Point3d(res) << std::endl;

	cv::Matx<double,3,3> S = H * P * H.t();  // + R
	cv::Matx<double,3,3> K = P * H.t() * S.inv();

	X += K*res;

	P = (I - K * H) * P;
}

double kalman::ray_trace(const double x, const double y, const double theta, const double range, const double angle) const{//const double y, const double theta, const double range, const double angle){
    //std::cout << "Range: " << range << " Angle: " << angle << std::endl;
    if(range >= 29.9)
        return 0;

	const cv::Point2i robot = this->toImage( cv::Point2d(x, y) ); //Get Robot's Esitimated position.

	const double laser_x = range * cos(theta + angle);
    const double laser_y = range * sin(theta + angle);

    const cv::Point2d stage_exp(x + laser_x, y + laser_y);
    const cv::Point2i map_expected = this->toImage(stage_exp);

    cv::LineIterator lit(this->map, robot, map_expected);
    if(lit.count == -1){
		std::cout << "LINE ERROR";
        return 0;
	}

	cv::Point2d actual;
    while(true){ //Follow line until hit occupied cell. Bounded the image in ctor so can't loop forever.
    	if(*(*lit) == OCCUPIED){
        	actual = this->toStage( lit.pos() ); // Difference between estimated and expected.
            break;
        }
    	lit++;
    }
	const double dx = (x - actual.x);
	const double dy = (y - actual.y);
	return sqrt(dx*dx + dy*dy);
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


cv::Mat kalman::show_map(const std::string& win_name, bool draw) const {
    cv::Mat clone = this->map.clone();

	const cv::Point2d p(X[0], X[1]);
//    cv::circle(clone, this->toImage(cv::Point2d(gt_x,gt_y)), 5, 155, -1);
    cv::circle(clone, this->toImage(p), 5, 10, -1);

    if(draw){
        cv::imshow(win_name, clone);
        cv::waitKey(10);
    }
	return clone;
}


void kalman::pose_callback(const nav_msgs::Odometry msg){
//    this->gt_x = -msg.pose.pose.position.y;
//    this->gt_y = msg.pose.pose.position.x;

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

//    this->gt_theta = heading;
}
