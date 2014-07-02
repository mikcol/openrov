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

#include <pcl/common/io.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace cv;
using namespace std;


Mat img;     	// Camera img
Mat greenImg;	// Green image
Mat bwImg;      // binary image

Mat top_roi,bottom_roi;


Point p_top, p_bottom, top_center, bottom_center;
float alpha, angle_increment, alpha_top, alpha_bottom, x_roi, y_roi, roi_width;
int roi_height, img_height, img_width, height, width;
Rect region_of_interest;
vector<Vec4i> top_lines,bottom_lines;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr top_cloud 		(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr bottom_cloud 	(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud 		(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlier_cloud 	(new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud 	(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud 	(new pcl::PointCloud<pcl::PointXYZRGB>);
// Initialize pointers to coefficients and inliers
pcl::ModelCoefficients::Ptr coefficients 		(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers 				(new pcl::PointIndices);

vector<pcl::PointIndices::Ptr> inline_vect;
vector<pcl::ModelCoefficients> coef_vect;

// Create the filtering object
pcl::ExtractIndices<pcl::PointXYZ> extract;

// Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;

ofstream planes_file;

int
simpleVis ()
{
	ROS_INFO("Started simpleVis thread");
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (0.1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(final_cloud);
	// Add cloud
	viewer->addPointCloud<pcl::PointXYZRGB> (final_cloud,rgb, "sample cloud");

	//viewer->addPointCloud<pcl::PointXYZ> (all_cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->initCameraParameters ();

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		viewer->updatePointCloud<pcl::PointXYZRGB> (final_cloud,rgb, "sample cloud");
		//viewer->updatePointCloud<pcl::PointXYZ> (all_cloud, "sample cloud");
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	return 0;

}

// Finds center of line
int find_center(vector<Vec4i> lines){
	//(y1+y2)/2
	if (lines.data()) {
		int center =(lines[0][1]+ lines[0][3])/2;
		return center;
	}
	else return -1;
};

// Calculates distance from line
int calc_dist(Point center, int height){
	int dist =  center.y;
	return dist;
};

void init_images(int width, int height){
	bwImg 		= Mat(height, width, IPL_DEPTH_8U, 1);
	greenImg 	= Mat(height, width, IPL_DEPTH_8U, 3);
};

void
find_planes(int img_no, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b){
	
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	
	// Set method and threshold
	seg.setMethodType (pcl::SAC_RANSAC);
	//seg.setDistanceThreshold (0.05);
	seg.setDistanceThreshold (0.02);
	seg.setInputCloud (temp_cloud);
	
	// Set the axis for which to search for perpendicular planes
	Eigen::Vector3f axis = Eigen::Vector3f(1.0,1.0,0.0);    // here specify the plane i.e X or Y or Z
	
	// Set a allowed deviation angle from vector axis
	seg.setEpsAngle((65*CV_PI)/180);
//	seg.setEpsAngle((45*CV_PI)/180);
	seg.setAxis(axis);

	temp_cloud->width = 81;
	temp_cloud->height = 2;
	temp_cloud->resize (top_cloud->width * top_cloud->height);
	// Fill the final point cloud with data points	
	*temp_cloud = *cloud_a;
	*temp_cloud += *cloud_b;
//	*all_cloud = *temp_cloud;

	int nr_points = temp_cloud->points.size();

	// Optional
	seg.setOptimizeCoefficients (true);
	

	// Counter for each plane found
	unsigned int i = 1;
	final_cloud->clear();

	while(temp_cloud->points.size() > 0.15 * nr_points && i < 7){
		
		// Segment the planes and insert the coefficients and inliers in vectors
		seg.segment (*inliers, *coefficients);
		inline_vect.push_back(inliers); 
		coef_vect.push_back(*coefficients);
		
		// Extract the inliers
		extract.setInputCloud (temp_cloud);
		extract.setIndices (inliers);

		// Mandatory: Init the size of the inlier_cloud
		inlier_cloud->width    = inliers->indices.size();
		inlier_cloud->height   = 1;
		inlier_cloud->resize (inlier_cloud->width * inlier_cloud->height);

		// Insert the inliers into the cloud and give them a color
		for(unsigned int j = 0; j < inliers->indices.size(); j++){
			inlier_cloud->points[j].x = temp_cloud->points[inliers->indices[j]].x;
			inlier_cloud->points[j].y = temp_cloud->points[inliers->indices[j]].y;
			inlier_cloud->points[j].z = temp_cloud->points[inliers->indices[j]].z;
			inlier_cloud->points[j].r = 255;
			inlier_cloud->points[j].g = 255*(i%2);
			inlier_cloud->points[j].b = 255*(i%3);
		}
		extract.setNegative(true);
		extract.filter(*temp_cloud);
	
		ROS_INFO("Run no: %d, points in cloud: %d , nr_points: %d", i,temp_cloud->points.size(),nr_points);	
		*final_cloud += *inlier_cloud;
		i++;
	}

	for(i = 0; i < coef_vect.size(); i++){
		planes_file << img_no << "\t" <<  coef_vect[i].values[0] << "\t" << coef_vect[i].values[1] << "\t" << coef_vect[i].values[2] << "\t" << coef_vect[i].values[3] << endl;
	}
	unsigned int size = coef_vect.size();
	for(i = 0; i < size; i++){
		coef_vect.pop_back();
	} 
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}

int main(int argc, char **argv)
{

	boost::thread viewer_thread(simpleVis);
	int n_rois = 81;	

	// Open file stream
	ofstream outputfile;
	outputfile.open("ranges.txt");
	planes_file.open("planes.txt");

	for(int img_no = 1; img_no < argc;img_no++){

		img = imread(argv[img_no]);

		Size s = img.size();
		width = s.width;
		height = s.height;

		init_images(width,height);
		// Check that image is loaded
		if(!img.data){ return -1;}

		cvtColor(img,img,CV_BGR2HSV);
		//inRange(img,Scalar(40,150,180),Scalar(80,255,255),greenImg);
		inRange(img,Scalar(40,200,10),Scalar(70,255,100),greenImg);

		threshold(greenImg,bwImg,1,255,THRESH_BINARY);

		// Create Binary image with a threshold value of 128
		//cvtColor(img,bwImg,CV_BGR2GRAY);
		//threshold(bwImg, bwImg, 1, 255.0, THRESH_BINARY);
		//ROS_INFO("TEST3");

		Mat bwclone = bwImg.clone();
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
		bwImg=skel;

		for (int j = 0; j < n_rois; j++) {

			// Set parameters for ROI
			roi_height = height/2;
			roi_width = width/n_rois;
			x_roi =j*roi_width;

			// Set and draw region of interest (TOP)
			region_of_interest = Rect(x_roi, 0, roi_width, roi_height );
			top_roi = bwImg(region_of_interest);
			
			// (BOTTOM)
			region_of_interest = Rect(x_roi, roi_height, roi_width, roi_height );
			bottom_roi = bwImg(region_of_interest);

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
			outputfile << img_no << "\t" << x_center << "\t" << top_dist << "\t" << bottom_dist << "\t";
			/* FILL CLOUD */
			// Mandatory: Init he sizes of the point clouds
			top_cloud->width    = n_rois;
			top_cloud->height   = 1;
			top_cloud->resize (top_cloud->width * top_cloud->height);

			bottom_cloud->width = top_cloud->width;
			bottom_cloud->height = top_cloud->height;
			bottom_cloud->resize (top_cloud->width * top_cloud->height);

			double x_f,y_f,z_b,y_b,x_b,focal_length,b,sensor_width,sensor_height;

			sensor_width = 0.032;
			sensor_height = 0.018;
			
			focal_length = 0.009238;
			b = 0.166;
			x_f = sensor_width/width*x_center;
			y_f = sensor_height/height*top_dist;
			
			x_b = b*focal_length/y_f;
			y_b = x_b*x_f/focal_length;
			z_b = 0.1;
			
			// Write world top sensor coordinates to ranges.txt
			outputfile << x_b << "\t" << y_b << "\t" << z_b << "\t";

			top_cloud->points[j].x = x_b;
			top_cloud->points[j].y = y_b;
			top_cloud->points[j].z = z_b;
			// Calculate the point (x_top,y_top)

			x_f = sensor_width/width*x_center;
			y_f = sensor_height/height*bottom_dist;
			
			x_b = b*focal_length/y_f;
			y_b = x_b*x_f/focal_length;
			z_b = -0.1;

			// Write world bottom sensor coordinates to ranges.txt
			outputfile << x_b << "\t" << y_b << "\t" << z_b << endl;
			
			// Insert points into clouds
			bottom_cloud->points[j].x = x_b;
			bottom_cloud->points[j].y = y_b;
			bottom_cloud->points[j].z = z_b;
		}
		find_planes(img_no,top_cloud,bottom_cloud);
	}
	outputfile.close();
	planes_file.close();
	return 0;
}
