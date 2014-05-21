#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "OpenROVmessages/LaserMsg.h"
#include "OpenROVmessages/PlanesData.h"
#include "OpenROVmessages/PlanesDataArray.h"

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

using namespace std;
using namespace cv;



// Point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);

// Initialize pointers to coefficients and inliers
pcl::ModelCoefficients::Ptr coefficients 		(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers 				(new pcl::PointIndices);
// Init array to store the normals of the found planes
vector<pcl::ModelCoefficients> coef_vect;

// Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;

// Create the filtering object
pcl::ExtractIndices<pcl::PointXYZ> extract;

// temporary point used to copy from one cloud to another
pcl::PointXYZ point;

// init variable
bool first_run = true;
int nr_points;

// Define the msg to be published
OpenROVmessages::PlanesData data;
OpenROVmessages::PlanesDataArray msg;
ros::NodeHandle n;
ros::Publisher pub = n.advertise<OpenROVmessages::PlanesDataArray>("PlaneNormals", 100);


void init(const OpenROVmessages::LaserMsg msg){

	/********************* INITIALIZE CLOUDS *********************/
	// Mandatory: Init he sizes of the point clouds
	temp_cloud->width = msg.n_rois;
	temp_cloud->height = 2;
	temp_cloud->resize(temp_cloud->width * temp_cloud->height);

	nr_points = temp_cloud->points.size();

	/**************** INITIALIZE SEGMENTER ****************/
	// Mandatory: use perpendicular plane to use setAxis() function
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	
	// Set method and threshold
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (5);
	seg.setInputCloud (temp_cloud);
	
	// Set the axis for which to search for perpendicular planes
	Eigen::Vector3f axis = Eigen::Vector3f(1.0,1.0,0.0);
	
	// Set a allowed deviation angle from vector axis
	seg.setEpsAngle((60*CV_PI)/180);
	seg.setAxis(axis);
}

void fill_cloud(const OpenROVmessages::LaserMsg msg){

	for(int i = 0; i < msg.n_rois; i++){
	
		point.x = msg.ranges_top[i];
		point.y = msg.ranges_center[i];
		point.z = 0.1;

		temp_cloud->push_back(point);

		point.x = msg.ranges_bottom[i];
		point.y = msg.ranges_center[i];
		point.z = -0.1;

		temp_cloud->push_back(point);
	}
}

void find_planes(const OpenROVmessages::LaserMsg msg){
	
	// Fill cloud with data
	fill_cloud(msg);

	// Find planes in the cloud
	while(temp_cloud->points.size() > 0.2 * nr_points){

		// Segment the planes and insert the coefficients and inliers in vectors
		seg.segment (*inliers, *coefficients);
		// push coeffients onto stack
		coef_vect.push_back(*coefficients);

		// Extract the inliers
		extract.setInputCloud (temp_cloud);
		extract.setIndices (inliers);

		// Add inliers to the final cloud
		for(unsigned int j = 0; j < inliers->indices.size(); j++){

			point = temp_cloud->points[ inliers->indices[j] ];
			//final_cloud->push_back( point );
		}
		// Extract found inliers from cloud
		extract.setNegative(true);
		extract.filter(*temp_cloud);
	}
}


void publish_msg(){
	// Fill Message
	for(unsigned int i = 0; i < coef_vect.size(); i++){
		data.a = (float)coef_vect[i].values[0];
		data.b = (float)coef_vect[i].values[1];
		data.c = (float)coef_vect[i].values[2];
		data.d = (float)coef_vect[i].values[3];
		
		msg.normals.push_back(data);
	}
	pub.publish(msg);
}

void laserCallback(const OpenROVmessages::LaserMsg msg){

	if(first_run){
		init(msg);
		first_run = false;
	}
	else
		find_planes(msg);
		publish_msg();
}


int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "find_planes");
	ros::NodeHandle n;

	// Subscribe to the LaserMsg
	ros::Subscriber sub = n.subscribe("laser", 100, laserCallback);

	// Run rosnode
	ros::spin();

	return 0;
}
