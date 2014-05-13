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
Point p_top,p_bottom;
float alpha;
float angle_increment;
int d_width;
int d_height;
/*****************************************
********** Functions *********************
*****************************************/

void calc_2d(const laserlines::LaserMsg msg){
	// Set the background image to refresh the imag

	background = imread("/home/nicholas/openrov/src/laserlines/resources/openrov_background.png");
//	cvSet(background,Scalar(255,255,255));

	angle_increment = msg.angle_span/msg.n_rois; // 90 deg / no_of_roi's
	// Calculate new point in 2D space
	for(int i = 0; i < msg.n_rois; i++){
	
		// Calculate the angle of the point in 2d space
		alpha = ((90-angle_increment)/2-i*angle_increment) * CV_PI/180; // ((90-angle_res)/2 - j*angle_res)*Pi/180

		ROS_INFO("Alpha: %f, angle_inc: %f",alpha,angle_increment);
		p_top.x = msg.ranges_top[i]*sin(alpha)/cos(alpha);
		p_top.y = -msg.ranges_top[i];
		ROS_INFO("(p_top.x,p_top.y): (%d,%d)",p_top.x,p_top.y);
		p_bottom.x = msg.ranges_bottom[i]*sin(alpha)/cos(alpha);
		p_bottom.y = -msg.ranges_bottom[i];

		circle(background, Point(msg.frame_width/2-p_top.x , msg.frame_height/2 + p_top.y), 2, Scalar(0,0,0));
		circle(background, Point(msg.frame_width/2-p_bottom.x , msg.frame_height/2 + p_bottom.y), 2, Scalar(255,255,0));


	}
	imshow("Laser Points", background);
	waitKey(30);
};

void chatterCallback(const laserlines::LaserMsg msg)
{
	calc_2d(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;

	namedWindow("Laser Points", CV_WINDOW_AUTOSIZE);

	ros::Subscriber sub = n.subscribe("chatter", 100, chatterCallback);

	ros::spin();

	return 0;
}
