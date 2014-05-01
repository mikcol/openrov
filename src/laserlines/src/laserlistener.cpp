#include "ros/ros.h"
#include "std_msgs/String.h"
#include "laserlines/LaserMsg.h"

void chatterCallback(const laserlines::LaserMsg msg)
{
  ROS_INFO("Ranges_top: [%d]", msg.ranges_top);
  ROS_INFO("Ranges_bottom: [%d]",msg.ranges_bottom);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();

  return 0;
}
