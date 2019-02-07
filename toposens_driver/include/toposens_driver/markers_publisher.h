#ifndef MARKERS_PUBLISHER_H
#define MARKERS_PUBLISHER_H

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sstream>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <toposens_msgs/TsScan.h>



struct Point {
	float x;
	float y;
	float z;
	float v;
	ros::Time stamp;
};


class MarkersPublisher
{
protected:
	ros::NodeHandle nh;
	ros::Publisher markers_pub;
	ros::Subscriber points_sub;
	std::vector<Point> newPoints;
	std::vector<Point> points;
	std::string frame;
	float lifetime;
	float volume_thresh;
	float min_dist;
	float volume_divider;
	float max_volume;
	float volume_offset;
	std::string color_direction;
	float color_min;
	float color_max;

public:
	MarkersPublisher();
	void pointsSubCallback(const toposens_msgs::TsScan::ConstPtr& msg);
	void updatePoints();
	void publishMarkers();
	std_msgs::ColorRGBA colorRainbow(float i);
	bool newFrame;
};

#endif
