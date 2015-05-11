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

Mat img;     	// Camera img
Mat grayImg;    // gray image for the conversion of the original image
Mat blurImg;    // gray flipped image
Mat invImg;     // Inverted image
Mat bwImg;      // binary image
Mat cdst;       // final image
Mat cannyImg;  	// Canny edge image
Mat greenImg;	// Image containing greens
Mat HSVImg;	// HSV color image
Mat errodeImg;


int roi_height,img_height,img_width;
int height,width;
float x_roi,y_roi,roi_width;
Rect region_of_interest;
Mat top_roi;
Mat bottom_roi;
vector<Vec4i> top_lines,bottom_lines;
Point top_center,bottom_center;

VideoCapture capture(0);

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
	/* HACK CODE*/
	img 		= Mat(height, width, IPL_DEPTH_8U, 1);
//	/* NORMAL CODE */
	img 		= Mat(height, width, IPL_DEPTH_8U, 3);
	HSVImg 		= Mat(height, width, IPL_DEPTH_8U, 3);
	greenImg 	= Mat(height, width, IPL_DEPTH_8U, 3);
	invImg 		= Mat(height, width, IPL_DEPTH_8U, 1);
	blurImg 	= Mat(height, width, IPL_DEPTH_8U, 1);
	bwImg 		= Mat(height, width, IPL_DEPTH_8U, 1);
	cannyImg 	= Mat(height, width, IPL_DEPTH_8U, 1);
//	errodeImg 	= Mat(height, width, IPL_DEPTH_8U, 1);
};

int find_ranges(OpenROVmessages::LaserMsg *msg){

	int32_t top_dists[msg->n_rois];
	int32_t bottom_dists[msg->n_rois];
	int32_t x_center[msg->n_rois];
//	width = msg->frame_width;
//	height = msg->frame_height;
	
	//img = imread("/home/nicholas/openrov/src/laserlines/resources/focal_9238_3m.png");
	img = imread("/home/nicholas/openrov/src/laserlines/resources/laser_path/0001.png");
	//img = imread("/home/nicholas/Downloads/range_finder_skelimg.png");

	Size s = img.size();
	ROS_INFO("Set image size: %dx%d",msg->frame_width,msg->frame_height);
	ROS_INFO("Actual image size: %dx%d",s.width,s.height);
	width = s.width;
	height = s.height;

	// Check that image is loaded
	if(!img.data){ return -1;}

	// Convert image to HSV
	cvtColor(img, HSVImg, CV_BGR2HSV);

	// Find greens in image
	inRange(HSVImg, Scalar(80/2,0,100), Scalar(140/2,255,255), greenImg);
//	inRange(HSVImg, Scalar(5,10,10), Scalar(70,255,255), greenImg);

	// Create Binary image with a threshold value of 128
	threshold(greenImg, bwImg, 1, 255.0, THRESH_BINARY);

	// Invert image
	bitwise_not(bwImg,invImg);

	// Blur image
	GaussianBlur(invImg, blurImg, Size(3,3),2,2);

	// Erode green lines
	Mat Kernel(Size(2, 2), CV_8UC1);
	erode(bwImg,errodeImg,Kernel);

	// Edge detect
	int sobel = 3;
	int lower_thres = 100;
	int upper_thres = 200;
	Canny(errodeImg, cannyImg, lower_thres, upper_thres,sobel);
	
	cvtColor(cannyImg, cdst, CV_GRAY2BGR);


	Mat bwclone = img.clone();
	Mat skel(img.size(),CV_8UC1,Scalar(0));
    Mat temp,eroded;
    Mat element = getStructuringElement(MORPH_CROSS,Size(3,3));
    bool done;
    do {
        erode(bwclone,eroded,element);
        dilate(eroded,temp,element);
        subtract(bwclone,temp,temp);
        bitwise_or(skel,temp,skel);
        eroded.copyTo(bwclone);

        done = (countNonZero(bwclone) == 0);
    }while(!done);
    cvtColor(skel, cdst, CV_GRAY2BGR);


	for (int j = 0; j < msg->n_rois; j++) {

		// Set parameters for ROI
		roi_height = height/2;
		roi_width = width/msg->n_rois;
		x_roi =j*roi_width;

		// Set and draw region of interest (TOP)
		region_of_interest = Rect(x_roi, 0, roi_width, roi_height );
		top_roi = skel(region_of_interest);
		//	top_roi = cannyImg(region_of_interest);
//		top_roi = bwImg(region_of_interest);
		rectangle(cdst, region_of_interest, Scalar(0,0,255), 1, 8, 0);
		// (BOTTOM)
		region_of_interest = Rect(x_roi, roi_height, roi_width, roi_height );
              bottom_roi = skel(region_of_interest);
//		bottom_roi = cannyImg(region_of_interest);
//		bottom_roi = bwImg(region_of_interest);
		rectangle(cdst, region_of_interest, Scalar(0,255,255), 1, 8, 0);

		// Find lines
		HoughLinesP(top_roi, top_lines, 1, CV_PI/180, 1, (int)width/msg->n_rois/3, 5 );
		HoughLinesP(bottom_roi, bottom_lines, 1, CV_PI/180, 1, (int)width/msg->n_rois/3, 5 );

		// Find the center of lines
		try {
			top_center = Point(x_roi+roi_width/2 , find_center(top_lines) );
		} catch (Point top_center) {
			cout << "Exception Thrown: Top";
		}
		try {
			bottom_center = Point(x_roi+roi_width/2 , find_center(bottom_lines) + roi_height);
		} catch (Point bottom_center) {
			cout << "Exception Thrown: Bottom";
		}

		// Calculate the distance
		x_center[j] = -(x_roi+(roi_width-width)/2); 	// -(center_of_roi - center_of_image) : to flip the signage
		top_dists[j] = height/2-top_center.y;
		bottom_dists[j] = bottom_center.y - height/2;

		// Draw Houghlines
		if (top_lines.data()) {
			line( cdst, Point(top_lines[0][0]+x_roi, top_lines[0][1]),
					Point(top_lines[0][2]+x_roi, top_lines[0][3]), Scalar(0,0,255), 3, 8 );
		}
		if (bottom_lines.data()) {
			line( cdst, Point(bottom_lines[0][0]+x_roi, bottom_lines[0][1]+roi_height),
					Point(bottom_lines[0][2]+x_roi, bottom_lines[0][3]+roi_height), Scalar(0,0,255), 3, 8 );
		}

		// Draw Center of lines
		circle(cdst, top_center, 3, Scalar(0,255,0),2);
		circle(cdst, bottom_center , 3, Scalar(0,255,0),2);
		line(cdst, top_center, Point(top_center.x, roi_height), Scalar(0,255,255),1,8);
		line(cdst, bottom_center, Point(bottom_center.x, roi_height), Scalar(0,0,255),1);

		imshow("Hough Lines",cdst);
		waitKey(30);
	}
	msg->ranges_center.assign(x_center,x_center+msg->n_rois);
	msg->ranges_top.assign(top_dists,top_dists+msg->n_rois);
	msg->ranges_bottom.assign(bottom_dists,bottom_dists+msg->n_rois);
	return 0;
}


int main(int argc, char **argv)
{
	if (!capture.isOpened()) {
		cout << "Error, did not open Camera" << endl;
		return -1;	
	}
	// init ROS node for the roscore
	ros::init(argc, argv, "Range_finder");

	// create node
	ros::NodeHandle n;

	// create publisher of type <laserlines::LaserMsg>, with name "chatter"
	ros::Publisher chatter_pub = n.advertise<OpenROVmessages::LaserMsg>("laser", 100);

	// Set update rate in Hz
	ros::Rate loop_rate(1);

	// Create msg
	OpenROVmessages::LaserMsg msg;

	// Set frame size down from 1080p to 640*480
	init_images(msg.frame_width,msg.frame_height);
//	init_images(1280,720);

	//img = imread(argv[1]);
	
	
	while (ros::ok())
	{
		// Capture image
//		capture >> img;

		// fill msg with data
		find_ranges(&msg);

		// Publish data
		chatter_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
}
