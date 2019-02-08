#include <ros/ros.h>

#include "toposens_markers/marker.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "markers_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

//  toposens_markers::Markers m(nh, private_nh);
 // TODO: use marker class only to format and parse points
  // subscribe and publish right here

  toposens_markers::MarkersPublisher markers_publisher;

  ros::Rate loop_rate(20);

  while (ros::ok()) {

    if (markers_publisher.newFrame == true) {
      markers_publisher.updatePoints();
      markers_publisher.publishMarkers();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}