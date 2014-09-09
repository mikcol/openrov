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

#include <boost/ref.hpp>
#include <boost/bind.hpp>

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
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud  (new pcl::PointCloud<pcl::PointXYZ>);

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
OpenROVmessages::PlanesDataArray planes_msg;

// Create global publisher
ros::Publisher pub;

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
	
//	seg.setOptimizeCoefficients (true);
	// Set method and threshold
	seg.setMethodType (pcl::SAC_RANSAC);
//	seg.setDistanceThreshold (msg.ransac_threshold);
	seg.setDistanceThreshold (5);
	
	// Set the axis for which to search for perpendicular planes
	Eigen::Vector3f axis = Eigen::Vector3f(1.0,1.0,0.0);
	
	// Set a allowed deviation angle from vector axis
	seg.setEpsAngle((60*CV_PI)/180);
	seg.setAxis(axis);
}

int fill_cloud(const OpenROVmessages::LaserMsg msg){

	ROS_INFO("Entered fill_cloud");
	for(int i = 0; i < msg.n_rois; i++){
		temp_cloud->points[2*i].x = (float)msg.ranges_top[i];
		temp_cloud->points[2*i].y = (float)msg.ranges_center[i];
		temp_cloud->points[2*i].z = 100;

		temp_cloud->points[2*i+1].x = (float)msg.ranges_bottom[i];
		temp_cloud->points[2*i+1].y = (float)msg.ranges_center[i];
		temp_cloud->points[2*i+1].z = 0;

	}
	return 0;
}

int find_planes(const OpenROVmessages::LaserMsg msg){
	ROS_INFO("Entered find_planes");
	// Fill cloud with data
	fill_cloud(msg);

	// Find planes in the cloud
	while(temp_cloud->points.size() > 0.1 * nr_points){

		seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);

		seg.setOptimizeCoefficients (true);
		// Set method and threshold
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (msg.ransac_threshold);

		// Set the axis for which to search for perpendicular planes
		Eigen::Vector3f axis = Eigen::Vector3f(1.0,0.0,0.0);

		// Set a allowed deviation angle from vector axis
		seg.setEpsAngle((60*CV_PI)/180);
		seg.setAxis(axis);
		// Segment the planes and insert the coefficients and inliers in vectors
		seg.setInputCloud (temp_cloud);
		seg.segment (*inliers, *coefficients);

		// Check that we found a plane from the cloud
		if (inliers->indices.size () == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}
		// push coeffients onto stack
		coef_vect.push_back(*coefficients);

		// Extract the inliers
		extract.setInputCloud (temp_cloud);
		extract.setIndices (inliers);

		// Extract found inliers from cloud
		extract.setNegative(true);
		extract.filter(*temp_cloud);
	}
	return 0;
}


void publish_msg(){

	// Fill Message
	for(unsigned int i = 0 ; i < coef_vect.size(); i--){
		data.a = (float)coef_vect[i].values[0];
		data.b = (float)coef_vect[i].values[1];
		data.c = (float)coef_vect[i].values[2];
		data.d = (float)coef_vect[i].values[3];
		
		ROS_INFO("a: %f, b: %f, c: %f, d: %f",data.a,data.b,data.c,data.d);
		planes_msg.normals.push_back(data);
	}
	// Publish msg
	pub.publish(planes_msg);

	ROS_INFO("planes_msg.size(): %d, coef_vect.size(): %d", planes_msg.normals.size(),coef_vect.size());
	// Empty vectors for reuse
	while(planes_msg.normals.size() > 0){
		planes_msg.normals.pop_back();	
		coef_vect.pop_back();
	}
	ROS_INFO("planes_msg.size(): %d, coef_vect.size(): %d", planes_msg.normals.size(),coef_vect.size());
}

void laserCallback(const OpenROVmessages::LaserMsg msg){

	if(first_run){
		init(msg);
		first_run = false;
	}
	else
		if(!find_planes(msg))
			publish_msg();
}


int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "find_planes");

	ros::NodeHandle n;
	// Create publisher
	pub = n.advertise<OpenROVmessages::PlanesDataArray>("PlaneNormals", 100);

	// Subscribe to the LaserMsg
	ros::Subscriber sub = n.subscribe("laser", 100, laserCallback);

	// Run rosnode
	ros::spin();

	return 0;
}
