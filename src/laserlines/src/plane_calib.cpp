#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "OpenROVmessages/LaserMsg.h" // Custom ROS message
using namespace std;
using namespace cv;

// checks if file exists
bool
file_check(const char *fileName)
{
    ifstream file(fileName);
    return (bool)file;
}

// Finds center of line
int 
find_center(vector<Vec4i> lines){
	//(y1+y2)/2
	if (lines.data()) {
		int center =(lines[0][1]+ lines[0][3])/2;
		return center;
	}
	else return -1;
};

// Skeletonize image
Mat
skeletonize(Mat* img){

	Mat bwclone = img->clone(); // Clone original image
	Mat skel(img->size(),CV_8UC1,Scalar(0)); // init skeleton image
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

   return skel;
}

// Takes input image and finds the laser points in each ROI of the image and returns these
int 
find_ranges(OpenROVmessages::LaserMsg *msg, Mat *sourceImg){

    // Init data arrays
	int32_t top_dists[msg->n_rois];
	int32_t bottom_dists[msg->n_rois];
	int32_t x_center[msg->n_rois];

    // Init ROI variables
    int u_roi,v_roi;
    Mat top_roi, bottom_roi;
    Rect region_of_interest;
    vector<Vec4i> top_lines,bottom_lines;
    Point p_top, p_bottom, top_center, bottom_center;

    // Init frame height and width
	int width = msg->frame_width;
	int height = msg->frame_height;

    // init ROI
    int roi_height = height/2;
    int roi_width = width/msg->n_rois;

    // Init images
    Mat img         = Mat(height, width, IPL_DEPTH_8U, 1);
    Mat cdst        = Mat(height, width, IPL_DEPTH_8U, 1);
	Mat HSVImg      = Mat(height, width, IPL_DEPTH_8U, 1);
	Mat greenImg    = Mat(height, width, IPL_DEPTH_8U, 1);
	Mat invImg      = Mat(height, width, IPL_DEPTH_8U, 1);
	Mat bwImg       = Mat(height, width, IPL_DEPTH_8U, 1);
	Mat blurImg     = Mat(height, width, IPL_DEPTH_8U, 1);
	Mat errodeImg   = Mat(height, width, IPL_DEPTH_8U, 1);

    // Set pointer to source image
    img = *sourceImg;

	// Check that image is loaded
	if(!img.data){ return -1;}

	// Convert image to HSV
	cvtColor(img, HSVImg, CV_BGR2HSV);

	// Find greens in image
	inRange(HSVImg, Scalar(80/2,0,100), Scalar(140/2,255,255), greenImg);

	// Create Binary image with a threshold value of 1
	threshold(greenImg, bwImg, 1, 255.0, THRESH_BINARY);

	// Invert image
	bitwise_not(bwImg,invImg);

	// Blur image
	GaussianBlur(invImg, blurImg, Size(3,3),2,2);

	// Erode green lines
	Mat Kernel(Size(2, 2), CV_8UC1);
	erode(bwImg,errodeImg,Kernel);

    // Skeletonize image
    Mat skel = skeletonize(&img);
    cvtColor(skel, cdst, CV_GRAY2BGR);

    // Loop over each ROI and find points in each
	for (int j = 0; j < msg->n_rois; j++) {

		// Set parameters for ROI
		roi_height = height/2;
		roi_width = width/msg->n_rois;
		u_roi =j*roi_width;

		// Set and draw region of interest (TOP)
		region_of_interest = Rect(u_roi, 0, roi_width, roi_height );
		top_roi = skel(region_of_interest);
		// (BOTTOM)
        region_of_interest = Rect(u_roi, roi_height, roi_width, roi_height );
        bottom_roi = skel(region_of_interest);

		// Find lines
		HoughLinesP(top_roi, top_lines, 1, CV_PI/180, 1, (int)width/msg->n_rois/3, 5 );
		HoughLinesP(bottom_roi, bottom_lines, 1, CV_PI/180, 1, (int)width/msg->n_rois/3, 5 );

		// Find the center of lines if they exist
		try {
			top_center = Point(u_roi+roi_width/2 , find_center(top_lines) );
		} catch (Point top_center) {
			cout << "Exception Thrown: Top";
		}
		try {
			bottom_center = Point(u_roi+roi_width/2 , find_center(bottom_lines) + roi_height);
		} catch (Point bottom_center) {
			cout << "Exception Thrown: Bottom";
		}

		// Calculate the distance
		x_center[j] = -(u_roi+(roi_width-width)/2); 	// -(center_of_roi - center_of_image) : to flip the signage
		top_dists[j] = height/2-top_center.y;
		bottom_dists[j] = bottom_center.y - height/2;


	}
	msg->ranges_center.assign(x_center,x_center+msg->n_rois);
	msg->ranges_top.assign(top_dists,top_dists+msg->n_rois);
	msg->ranges_bottom.assign(bottom_dists,bottom_dists+msg->n_rois);
	return 0;
}

int 
main(int argc, char **argv)
{

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
    
    // Body frame distance from object
    int x_body;     

    // Calibration from camera
    if (argv[1] == std::string("-c"))
    {
        cout << "Running Calibration from camera: Please place robot in a 90 degree angle from a wall" << endl;
        cout << "Enter distance from wall (cm): ";
        cin >> x_body;
        cout << endl;
    }

    // Calibration from image
    if (argv[1] == std::string("-i"))
    {
        // if no filename given, ask for one
        char fileName;
        if (!argv[2])
        {
           cout << "Please enter a file name: ";
           cin >> fileName;
           cout << endl;
        }
        else
            fileName = *argv[2];

        // Check if file exsist
        if (!file_check(&fileName))
        {
            cout << "ERROR: File not found" << endl;
            return -1;
        }
        cout << "Running Calibration on file: " << fileName << endl;
        cout << "Enter distance from wall(cm): ";
        cin >> x_body;
        cout << endl;

        // TODO: fix this conversion
        Mat img = imread((const char *) fileName);
        find_ranges(&msg, &img);
    }
	//while (ros::ok())
	//{

	//	// fill msg with data
	//	find_ranges(&msg);

	//	// Publish data
	//	chatter_pub.publish(msg);

	//	ros::spinOnce();
	//	loop_rate.sleep();
	//}
    return 0;
}
