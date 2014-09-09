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
<<<<<<< HEAD
	stringstream ss;
=======
	Stringstream ss;
>>>>>>> f69c5de8c60ea7db1bbe22b48721a395da120470
    
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
	ros::Rate loop_rate(2);

	// Create msg
	OpenROVmessages::LaserMsg msg;

	int count = 0;
<<<<<<< HEAD
	String name,run_no;
=======
	String name, run_no;
>>>>>>> f69c5de8c60ea7db1bbe22b48721a395da120470

	while (ros::ok())
	{
		// Capture image
		capture >> image;
<<<<<<< HEAD

=======
>>>>>>> f69c5de8c60ea7db1bbe22b48721a395da120470
		ss << count;
		ss >> run_no;

		name = "~/openrov/pictures/" + run_no + ".png";
		// Save image
		imwrite(name, image);

		// Publish data
		chatter_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
}
