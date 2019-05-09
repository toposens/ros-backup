#include <ros/ros.h>

#include "toposens_pointcloud/mapping.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ts_cloud_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Rate loop_rate(10); // 10 Hz
  int counter = 1;
  toposens_pointcloud::Mapping m(nh, private_nh);

  while (counter++) {
    ros::spinOnce();
    loop_rate.sleep();
    // save every 10 seconds
    if (!(counter % 100)) m.save("toposens");
  }

  return 0;
}