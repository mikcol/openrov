#include <iostream>
#include <fstream>
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


int roi_height,img_height,img_width;
int height,width;
float x_roi,y_roi,roi_width;
Rect region_of_interest;
Mat top_roi;
Mat bottom_roi;
vector<Vec4i> top_lines,bottom_lines;
Point top_center,bottom_center;


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
	HSVImg 		= Mat(height, width, IPL_DEPTH_8U, 3);
	greenImg 	= Mat(height, width, IPL_DEPTH_8U, 3);
	invImg 		= Mat(height, width, IPL_DEPTH_8U, 1);
	blurImg 	= Mat(height, width, IPL_DEPTH_8U, 1);
	bwImg 		= Mat(height, width, IPL_DEPTH_8U, 1);
	cannyImg 	= Mat(height, width, IPL_DEPTH_8U, 1);
};



int main(int argc, char **argv)
{
	// Set frame size down from 1080p to 640*480

	int n_rois = 81;	


	// Open file stream
	ofstream outputfile;
	outputfile.open("data.txt");

	for(int img_no = 1; img_no < argc;img_no++){

		img = imread(argv[img_no]);

		Size s = img.size();
		width = s.width;
		height = s.height;

		init_images(width,height);
		// Check that image is loaded
		if(!img.data){ return -1;}

		// Convert image to HSV
		cvtColor(img, HSVImg, CV_BGR2HSV);

		// Find greens in image
		inRange(HSVImg, Scalar(5,10,10), Scalar(70,255,255), greenImg);

		// Invert image
		bitwise_not(greenImg,invImg);

		// Edge detect
		Canny(invImg, cannyImg, 50, 300);

		// Create Binary image with a threshold value of 128
		threshold(cannyImg, bwImg, 1, 255.0, THRESH_BINARY);
		cvtColor(bwImg, cdst, CV_GRAY2BGR);

		for (int j = 0; j < n_rois; j++) {

			// Set parameters for ROI
			roi_height = height/2;
			roi_width = width/n_rois;
			x_roi =j*roi_width;

			// Set and draw region of interest (TOP)
			region_of_interest = Rect(x_roi, 0, roi_width, roi_height );
			top_roi = bwImg(region_of_interest);
			rectangle(cdst, region_of_interest, Scalar(0,0,255), 1, 8, 0);
			// (BOTTOM)
			region_of_interest = Rect(x_roi, roi_height, roi_width, roi_height );
			bottom_roi = bwImg(region_of_interest);
			rectangle(cdst, region_of_interest, Scalar(0,255,255), 1, 8, 0);

			// Find lines
			HoughLinesP(top_roi, top_lines, 1, CV_PI/180, 5, (int)width/n_rois/3, 5 );
			HoughLinesP(bottom_roi, bottom_lines, 1, CV_PI/180, 5, (int)width/n_rois/3, 5 );

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
			int x_center = -(x_roi+(roi_width-width)/2); 	// -(center_of_roi - center_of_image) : to flip the sign
			int top_dist = height/2-top_center.y;
			int bottom_dist = bottom_center.y - height/2;

			// Write output to file
			outputfile << img_no << "\t" << x_center << "\t" << top_dist << "\t" << bottom_dist << endl;
		}
	}
	outputfile.close();
	return 0;
}
