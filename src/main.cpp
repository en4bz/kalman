#include <iostream>
#include <opencv2/core/core.hpp>

#include "kalman.hpp"

#include "ros/ros.h"

const std::string name = "Kalman";

int main(int argc, char *argv[]){
    ros::init(argc, argv, name);
    ros::NodeHandle node;
    if(argc != 2)
        std::cout << "No Input Image!" << std::endl;

    const cv::Mat map = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    const int spin_rate = 5;

	cv::namedWindow(name);

    kalman k(node,map,12,12,0,spin_rate);

    ros::Rate rate(spin_rate);
    while( ros::ok() ){
        ros::spinOnce();
		k.correct();
		k.show_map(name,true);
        rate.sleep();
    }
    ros::shutdown();

    return 0;
}
