#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "laserlines/LaserMsg.h"

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

/*****************************************
************ GLOBAL VARIABLES ************
*****************************************/

Point p_top,p_bottom;
float alpha,angle_increment,alpha_top,alpha_bottom;
unsigned int i;
// PCL Variables
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud 	(new pcl::PointCloud<pcl::PointXYZRGB>);

// initialize temporary, inlier, bottom & top point clouds
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud 		(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlier_cloud 	(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr top_cloud 		(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr bottom_cloud 	(new pcl::PointCloud<pcl::PointXYZ>);

// Initialize pointers to coefficients and inliers
pcl::ModelCoefficients::Ptr coefficients 		(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers 				(new pcl::PointIndices);

// Create the filtering object
pcl::ExtractIndices<pcl::PointXYZ> extract;

// Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;

/*****************************************
********** Functions *********************
*****************************************/

void init(){
	
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	
	// Set method and threshold
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (5);
	seg.setInputCloud (temp_cloud);
	
	// Set the axis for which to search for perpendicular planes
	Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);    // here specify the plane i.e X or Y or Z
	
	// Set a allowed deviation angle from vector axis
	seg.setEpsAngle((40*CV_PI)/180);
	seg.setAxis(axis);
}


int
//simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
simpleVis ()
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (50.0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(final_cloud);
	// Add cloud
//	viewer->addPointCloud<pcl::PointXYZ> (temp_cloud, "sample cloud");
	viewer->addPointCloud<pcl::PointXYZRGB> (final_cloud,rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->initCameraParameters ();
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
//		viewer->updatePointCloud<pcl::PointXYZ> (temp_cloud, "sample cloud");
		viewer->updatePointCloud<pcl::PointXYZRGB> (final_cloud,rgb, "sample cloud");
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	return 0;

}

void
find_planes(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b){
	
	// Initialize
	vector<pcl::PointIndices::Ptr> inline_vect;
	vector<pcl::ModelCoefficients> coef_vect;

	// Fill the final point cloud with data points	
	*temp_cloud = *cloud_a;
	*temp_cloud += *cloud_b;

	int nr_points = temp_cloud->points.size();

	
	// Optional
//	seg.setOptimizeCoefficients (true);
	

	// Counter for each plane found
	i = 1;

	while(temp_cloud->points.size() > 0.2 * nr_points){
		
		ROS_INFO("Run no: %d, points in cloud: %d , nr_points: %d", i,temp_cloud->points.size(),nr_points);	
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
	
		*final_cloud += *inlier_cloud;
		i++;
	}

	for(i = 0; i < coef_vect.size(); i++){
		ROS_INFO("Coef[%d]: a:%f, b:%f ,c:%f, d:%f", i,coef_vect[i].values[0],coef_vect[i].values[1],coef_vect[i].values[2],coef_vect[i].values[3]);
	}
}

 
int first_run = 1;
void calc_2d(const laserlines::LaserMsg msg){
	
	if(first_run){
		ofstream outputfile;
		outputfile.open("data.txt");
		for(int i = 0; i < msg.n_rois;i++){
			outputfile << msg.ranges_top[i] << "\t" << msg.ranges_bottom[i] << endl;
		}	
		outputfile.close();
	}
	// Mandatory: Init he sizes of the point clouds
	top_cloud->width    = msg.n_rois;
	top_cloud->height   = 1;
	top_cloud->resize (top_cloud->width * top_cloud->height);

	bottom_cloud->width = top_cloud->width;
	bottom_cloud->height = top_cloud->height;
	bottom_cloud->resize (top_cloud->width * top_cloud->height);

	// Calculate the angle increment value between each measured point
	angle_increment = msg.angle_span/msg.n_rois; // 90 deg / no_of_roi's

	// Calculate new point in 2D space
	for(int i = 0; i < msg.n_rois; i++){
	
		// Calculate the angle of the point in 2d space
		alpha = ((msg.angle_span-angle_increment)/2-i*angle_increment) * CV_PI/180; // ((90-angle_res)/2 - j*angle_res)*Pi/180
		//alpha_top = atan(msg.ranges_center[i]/msg.ranges_top[i]);
		//alpha_bottom = atan(msg.ranges_center[i]/msg.ranges_bottom[i]);

		// Calculate the point (x_top,y_top)
		p_top.x = msg.ranges_top[i]*atan(alpha);
		p_top.y = msg.ranges_top[i];

		// Calculate the point (x_bottom,y_bottom)
		p_bottom.x = msg.ranges_bottom[i]*atan(alpha);
		p_bottom.y = msg.ranges_bottom[i];

		// Insert points into clouds
		top_cloud->points[i].x = p_top.x;
		top_cloud->points[i].y = p_top.y;
		top_cloud->points[i].z = 100;
		bottom_cloud->points[i].x = p_bottom.x;
		bottom_cloud->points[i].y = p_bottom.y;
		bottom_cloud->points[i].z = 0;
	}
	// Find the planes in the cloud;
	find_planes(bottom_cloud,top_cloud);	
};

// Callback function for the msg
void chatterCallback(const laserlines::LaserMsg msg)
{
	calc_2d(msg);
}

int main(int argc, char **argv)
{
	// Start a thread with the 3D viewer
	boost::thread viewer_thread(simpleVis);

	init();
	// Init ROS
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;

	// Subscribe to the LaserMsg
	ros::Subscriber sub = n.subscribe("chatter", 100, chatterCallback);

	ros::spin();

	return 0;
}
