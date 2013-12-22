#include "kalman.hpp"

kalman::kalman(ros::NodeHandle& nh, const cv::Mat& pmap, double x_init, double y_init, double theta_init, int spin_rate)
	: map(pmap.clone()), dt(1.0/(double)spin_rate), linear(0), angular(0) {

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

	this->P(0,0) = 1;
	this->P(1,1) = 1;
	this->P(2,2) = 1;

	this->Q(0,0) = 1/10000.0;
	this->Q(1,1) = 1/10000.0;
	this->Q(2,2) = 1/10000.0;

    //Bound image by occupied cells.
    this->map.row(0) = cv::Scalar(0);
    this->map.row(map.size().width - 1) = cv::Scalar(0);
    this->map.col(0) = cv::Scalar(0);
    this->map.col(map.size().height - 1)  = cv::Scalar(0);
}


void kalman::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
 	const int size = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    std::cout << "Size: " << size << std::endl;
    int step = 100;

    laser.clear();

    for(int i = 0; i < size; i+= step){
        const double angle = msg->angle_min + msg->angle_increment * i;
    	laser.push_back( rangle(msg->ranges[i], angle) );
    }
    return;
}

void kalman::predict(const nav_msgs::Odometry msg){
	const double scale = 1 / 100.0;

	static boost::mt19937 rng;
	static boost::normal_distribution<double> x_noise;
	static boost::normal_distribution<double> y_noise;
	static boost::normal_distribution<double> theta_noise;

	static boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > dx(rng,x_noise);
	static boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > dy(rng,y_noise);
	static boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > dtheta(rng,theta_noise);

    double theta = X[2];

	this->X[0] += linear * dt * cos( X[2] ) + dx() * scale;
	this->X[1] += linear * dt * sin( X[2] ) + dy() * scale;
	this->X[2] += angular * dt + dtheta() * scale;

    theta = X[2];

//	std::cout << "X" << cv::Point3d(X) << std::endl;

	this->F(0,2) = -linear * dt * sin( theta ); //t+1 ?
	this->F(1,2) =  linear * dt * cos( theta ); //t+1 ?

//	std::cout << "F" << std::endl << cv::Mat(F) << std::endl;
//	std::cout << "P" << std::endl << cv::Mat(P) << std::endl;
	P = F * P * F.t() + Q;
	P = (P + P.t()) * 0.5;

//	std::cout << "P" << std::endl << cv::Mat(P) << std::endl;


	this->linear = msg.twist.twist.linear.x;
	this->angular = msg.twist.twist.angular.z;
	return;
}

double kalman::partial_x(double x, double y, double angle, double dx) const{
    double px = -ray_trace(x+2*dx, y, angle) + 8*ray_trace(x+dx, y, angle) - 8*ray_trace(x-dx, y, angle) + ray_trace(x-2*dx, y, angle);
    px /= 12*dx;
    return px;
}

double kalman::partial_y(double x, double y, double angle, double dy) const{
    double py = -ray_trace(x,y+2*dy, angle) + 8*ray_trace(x, y+dy, angle) - 8*ray_trace(x, y-dy, angle) + ray_trace(x, y-2*dy, angle);
    py /= 12*dy;
    return py;
}

double kalman::partial_theta(double x, double y, double angle, double dtheta) const{
    double ptheta = -ray_trace(x,y, angle + 2*dtheta) + 8*ray_trace(x, y, angle + dtheta)
                 - 8*ray_trace(x, y, angle-dtheta) + ray_trace(x, y, angle -2*dtheta);
    ptheta /= 12*dtheta;
    return ptheta;
}

void kalman::correct(){
    std::cout << "[" << ros::Time::now() << "]" << std::endl;
	const double x = this->X[0];
	const double y = this->X[1];
	const double theta = this->X[2];
	const double dx = 0.2; //Good results with 0.5, 0.5, pi/12;
	const double dy = 0.2;
	const double dtheta = CV_PI/21;

	cv::Vec<double,10> res; //5
    cv::Matx<double,10,3> H; //5,3

	for(size_t i = 0; i < laser.size(); i++){
		const double angle = laser[i].angle;
		res[i] = laser[i].range - ray_trace(x,y,theta+angle);
	    H(i,0) = partial_x(x, y, theta+angle, dx);
		H(i,1) = partial_y(x, y, theta+angle, dy);
//        H(i,2) = partial_theta(x,y,theta+angle,dtheta);
		H(i,2) = (ray_trace(x,y,theta+dtheta+angle) - ray_trace(x,y,theta-dtheta+angle) ) / 2*dtheta;
	}

	std::cout << std::endl << "res: [";
    for(int i = 0; i < 5; i++)
        std::cout << (res[i]) << ", ";
	std::cout << "]" << std::endl << "H" << cv::Mat(H) << std::endl;

	cv::Matx<double,10,10> S = H * P * H.t();  // + R //5,5
	cv::Matx<double,3,10> K = P * H.t() * S.inv(); //3,5

	std::cout << std::endl << "K" << cv::Mat(K) << std::endl;

	X += K*res;

//	std::cout << "X" << cv::Point3d(X) << std::endl;

	P = (I - K * H) * P;
	P = (P + P.t()) * 0.5;
//	std::cout << "P" << std::endl << cv::Mat(P) << std::endl;
}

double kalman::ray_trace(const double x, const double y, const double angle) const{//const double y, const double theta, const double range, const double angle){
    //std::cout << "Range: " << range << " Angle: " << angle << std::endl;
	const double range = 3.0;

	const cv::Point2i robot = this->toImage( cv::Point2d(x, y) ); //Get Robot's Esitimated position.

	const double laser_x = range * cos(angle);
    const double laser_y = range * sin(angle);

    const cv::Point2d stage_exp(x + laser_x, y + laser_y);
    const cv::Point2i map_expected = this->toImage(stage_exp);

    cv::LineIterator lit(this->map, robot, map_expected);
    if(lit.count == -1 || lit.step == 0 || lit.elemSize == 0){
		std::cout << "LINE ERROR";
        return range;
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

    const double llen = 1;

    const cv::Point2d gt(gt_x, gt_y);
	const cv::Point2d est(X[0], X[1]);
    const cv::Point2d gt_line(llen*cos(gt_theta),llen*sin(gt_theta));
    const cv::Point2d est_line(llen*cos(X[2]),llen*sin(X[2]));
    cv::circle(clone, this->toImage(gt), 5, 155, -1);
    cv::circle(clone, this->toImage(est), 5, 10, -1);
    cv::line(clone, toImage(gt), toImage(gt + gt_line), 100);
    cv::line(clone, toImage(est), toImage(est + est_line), 55);

    if(draw){
        cv::imshow(win_name, clone);
        cv::waitKey(10);
    }
	return clone;
}


void kalman::pose_callback(const nav_msgs::Odometry msg){
    this->gt_x = -msg.pose.pose.position.y;
    this->gt_y = msg.pose.pose.position.x;

    btScalar roll, pitch, heading;

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
//    this->X[2] = heading;
}
