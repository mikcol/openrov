#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "laserlines/LaserMsg.h"


using namespace std;
using namespace cv;

/*****************************************
************ GLOBAL VARIABLES ************
*****************************************/

Mat background = Mat(640, 480, IPL_DEPTH_8U,3);

/*****************************************
********** Functions *********************
*****************************************/

void calc_2d(const laserlines::LaserMsg msg){

	ROS_INFO("Called the calc_2d function");
	// Set the background image to refresh the image
	background.setTo(Scalar(255,255,255));
	ROS_INFO("Set the background color");
//	cvSet(background,Scalar(255,255,255));
	int d_width = 640;
	int d_height = 480;

	// Calculate new point in 2D space
	Point p_top,p_bottom;
	for(int i = 0; i < msg.ranges_top.size(); i++){
		float alpha = (45-i*msg.angle_increment) * CV_PI/180; // (60 - j* degree_resolution)*Pi/180
	
		p_top.x = msg.ranges_top[i]*sin(alpha);
		p_top.y = -msg.ranges_top[i];
		p_bottom.x = msg.ranges_bottom[i]*sin(alpha);
		p_bottom.y = -msg.ranges_bottom[i];

		circle(background, Point(d_width/2-p_top.x , d_height/2 + p_top.y), 2, Scalar(0,0,255));
		circle(background, Point(d_width/2-p_bottom.x , d_height/2 + p_bottom.y), 2, Scalar(255,255,0));
	}
	imshow("Laser Points", background);
};

void chatterCallback(const laserlines::LaserMsg msg)
{
	ROS_INFO("Ranges_top: [%d]", msg.ranges_top[0]);
	ROS_INFO("Ranges_bottom: [%d]",msg.ranges_bottom[0]);
	calc_2d(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	namedWindow("Laser Points", CV_WINDOW_AUTOSIZE);
	
	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	ros::spin();

	return 0;
}
