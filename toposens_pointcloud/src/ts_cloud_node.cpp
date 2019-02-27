#include <ros/ros.h>

#include "toposens_pointcloud/cloud.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ts_cloud_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  toposens_pointcloud::Cloud c(nh, private_nh);
  ros::spin();
  ROS_ERROR_STREAM("HELLLOOOO");

  return 0;
}