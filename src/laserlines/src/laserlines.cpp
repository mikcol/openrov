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

  int count = 0;

  // OpenCV
  Mat img;
  img = imread("")


  while (ros::ok())
  {
    // Create msg
    laserlines::LaserMsg msg;

    // fill msg with data
    float max_angle = 10;
    float min_angle = 0;
    float range[10];
    msg.angle_max = max_angle;

    ROS_INFO("%f", msg.angle_max);
    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
