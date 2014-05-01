#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "laserlines/LaserMsg.h"
#include <sstream>
using namespace cv;
using namespace std;

Mat img;     // original image
Mat grayImg;    // gray image for the conversion of the original image
Mat blurImg;    // gray flipped image
Mat invImg;     // Inverted image
Mat bwImg;      // binary image
Mat cdst;       // final image
Mat cannyImg;
Mat greenImg;
Mat HSVImg;


int n_rois = 10;
int n_rois_max = 50;
int n_rois_min = 10;
int roi_slider;
int roi_height ;
float x_roi,y_roi,width_roi;



// Finds center of line
int find_center(vector<Vec4i> lines){
	if (lines.data()) {
		int center =(lines[0][1]+ lines[0][3])/2;
		return center;

	}
	else return -1;
};

// Calculates distance from line
int calc_dist(Point center, int height){
	//dist = (y1+y2)/2 - center_of_pic
	int dist =  center.y;
	return dist;
};

void init_images(int width, int height){
	HSVImg = Mat(height, width, IPL_DEPTH_8U, 3);
	greenImg = Mat(height, width, IPL_DEPTH_8U, 3);
	invImg = Mat(height, width, IPL_DEPTH_8U, 1 );
	blurImg = Mat(height, width, IPL_DEPTH_8U, 1 );
	bwImg = Mat(height, width, IPL_DEPTH_8U, 1 );
};

laserlines::LaserMsg find_ranges(Mat& newImg,int width, int height){

	laserlines::LaserMsg msg;


	//capture.set(CV_CAP_PROP_FRAME_WIDTH, width);
	//capture.set(CV_CAP_PROP_FRAME_HEIGHT, height);

	cv::Rect roi( cv::Point( 640/2-60/2, 480/2-86/2 ), cv::Size( 60, 86 ));

	// Convert image to HSV
	cvtColor(newImg, HSVImg, CV_BGR2HSV);

	// Find greens in image
	inRange(HSVImg, Scalar(80/2,100,100), Scalar(140/2,255,255), greenImg);

	// Invert image
	bitwise_not(greenImg,invImg);

	// flip image
	GaussianBlur(greenImg, blurImg, Size(3,3),2,2);

	// Edge detect
	Canny(blurImg, cannyImg, 50, 300);

	// Create Binary image with a threshold value of 128
	threshold(cannyImg, bwImg, 128, 255.0, THRESH_BINARY);
	cvtColor(bwImg, cdst, CV_GRAY2BGR);
	
	for (int j = 0; j < n_rois; j++) {

		// Set parameters for ROI
		roi_height = height/2;
		width_roi = width/n_rois;
		x_roi =j*width_roi;

		// Set and draw region of interest (TOP)
		Rect region_of_interest = Rect(x_roi, 0, width_roi, roi_height );
		Mat top_roi = bwImg(region_of_interest);
		rectangle(cdst, region_of_interest, Scalar(0,0,255), 1, 8, 0);
		// (BOTTOM)
		region_of_interest = Rect(x_roi, height/2, width_roi, roi_height );
		Mat bottom_roi = bwImg(region_of_interest);
		rectangle(cdst, region_of_interest, Scalar(0,255,255), 1, 8, 0);

		// Find lines
		vector<Vec4i> top_lines,bottom_lines;
		HoughLinesP(top_roi, top_lines, 1, CV_PI/180, 5, (int)width/n_rois/3, 5 );
		HoughLinesP(bottom_roi, bottom_lines, 1, CV_PI/180, 5, (int)width/n_rois/3, 5 );

		// Find the center of lines
		Point top_center,bottom_center;
		try {
			top_center = Point(x_roi+width_roi/2 , find_center(top_lines) );
		} catch (Point top_center) {
			cout << "Exception Thrown: Top";
		}
		try {
			bottom_center = Point(x_roi+width_roi/2 , find_center(bottom_lines) + roi_height);
		} catch (Point bottom_center) {
			cout << "Exception Thrown: Bottom";
		}

		// Calculate the distance
		int top_dist = calc_dist(top_center,height);
		int bottom_dist = calc_dist(Point(bottom_center.x,height-bottom_center.y), height);
		//ROS_INFO("Calculated all the distances as top: %d, bot: %d",top_dist,bottom_dist);
	//	ROS_INFO("Calculated all the distances as top");
		msg.ranges_top[j] = top_dist;
		msg.ranges_bottom[j] = bottom_dist;
		//ROS_INFO("set all the distances");
	}
	msg.angle_increment = 6;
	msg.n_rois = 10;
	return msg;
}


int main(int argc, char **argv)
{
	// init ROS node for the roscore
	ros::init(argc, argv, "talker");

	// create node
	ros::NodeHandle n;

	// create publisher of type <laserlines::LaserMsg>, with name "chatter"
	ros::Publisher chatter_pub = n.advertise<laserlines::LaserMsg>("chatter", 100);

	// Set update rate in Hz
	ros::Rate loop_rate(1);

	// Load image into img
	img = imread("/home/nicholas/openrov/src/laserlines/resources/laser_lines.png");

	// Set frame size down from 1080p to 640*480
	Size s = img.size();
	init_images(s.width,s.height);

	if(img.data){
		while (ros::ok())
		{
			// Create msg
			laserlines::LaserMsg msg;

			// fill msg with data
			msg = find_ranges(img,s.width,s.height);

		//	ROS_INFO("top_ranges: %d", msg.ranges_top[0]);
		//	ROS_INFO("bottom_ranges: %d",msg.ranges_bottom[0]);
			// Publish data
			chatter_pub.publish(msg);

			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	else return 0;
}
