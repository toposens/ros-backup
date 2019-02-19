#include <ros/ros.h>

#include "toposens_markers/marker.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ts_markers_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  toposens_markers::Marker m(nh, private_nh);
  ros::spin();

  return 0;
}