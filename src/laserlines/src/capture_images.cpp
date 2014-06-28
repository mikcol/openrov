#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "OpenROVmessages/LaserMsg.h"
#include <sstream>
#include <string>
using namespace cv;
using namespace std;


int main(int argc, char **argv)
{
	VideoCapture capture(0);
	Mat image;
    
	if (!capture.isOpened()) {
		cout << "Error, did not open Camera" << endl;
		return -1;	
	}
	// init ROS node for the roscore
	ros::init(argc, argv, "Image Capturer");

	// create node
	ros::NodeHandle n;

	// create publisher of type <laserlines::LaserMsg>, with name "chatter"
	ros::Publisher chatter_pub = n.advertise<OpenROVmessages::LaserMsg>("laser", 100);

	// Set update rate in Hz
	ros::Rate loop_rate(10);

	// Create msg
	OpenROVmessages::LaserMsg msg;
	
	int count = 0;
    String name;
    
	while (ros::ok())
	{
		// Capture image
		capture >> img;
        
        name = "~/openrov/pictures/" + to_string(count) + ".png";
		// Save image
        imwrite(name, bwImg);

		// Publish data
		chatter_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
}
