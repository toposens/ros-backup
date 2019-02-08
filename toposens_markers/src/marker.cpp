#include "toposens_markers/marker.h"

#include <toposens_msgs/TsScan.h>

namespace toposens_markers
{

float _sensingRange = 0;

MarkersPublisher::MarkersPublisher()
{
	ros::NodeHandle private_nh("~");

	// Get/Set parameters
	private_nh.param<std::string>("frame_id", frame, "toposens");		//Frame for tf
	private_nh.param<float>("lifetime", lifetime, 0.5); 								//Period of time for PCL
//	private_nh.param<float>("volume_thresh", volume_thresh, 80.0);			//Minimum volume a point needs to be processed
//	private_nh.param<float>("min_dist", min_dist, 0.15);
	private_nh.param<float>("volume_divider", volume_divider, 100.0);
	private_nh.param<float>("max_volume", max_volume, 20.0);
	private_nh.param<float>("volume_offset", volume_offset, 5.0);


	private_nh.param<std::string>("color_direction", color_direction, "z");
	private_nh.param<float>("color_min", color_min, 0.0);
	private_nh.param<float>("color_max", color_max, 0.3);

	// Subscribe to topic with ts alpha points
	points_sub = nh.subscribe("ts_points", 100, &MarkersPublisher::pointsSubCallback, this);

	// Advertise topic for Pointcloud-Messages
	markers_pub = nh.advertise<visualization_msgs::MarkerArray>("ts_markers", 100);
}


void MarkersPublisher::pointsSubCallback(const toposens_msgs::TsScan::ConstPtr& msg)
{

	newPoints.clear();
	for (int i = 0; i < msg->points.size(); i++) {
		struct Point point;
		point.x = msg->points[i].x;
		point.y = msg->points[i].y;
		point.z = msg->points[i].z;
		point.v = msg->points[i].v;
		point.stamp = msg->header.stamp;
		newPoints.push_back(point);

		//ROS_WARN("x = %f, v = %f", newPoints[0].x, newPoints[0].v);
	}
	newFrame = true;
}


void MarkersPublisher::updatePoints() // Update vector of points
{
	newFrame = false;

	// insert new points
	for (int i = 0; i < newPoints.size(); i++)
	{
		// ROS_INFO("%f", newPoints[i].x);
	//	if (newPoints[i].x >= min_dist && newPoints[i].v >= volume_thresh) {
			points.push_back(newPoints[i]);
      if (newPoints[i].x > _sensingRange) {
        _sensingRange = newPoints[i].x;
      }
	//	}
	//	else {
	//		ROS_WARN("x = %f, v = %f", newPoints[i].x, newPoints[i].v);
	//	}
	}

	// delete outdated points
	if (points.size() > 0)
	{

	//ROS_INFO_STREAM(points[0].stamp);
		ros::Time currentTime = ros::Time::now();
		while (currentTime - points[0].stamp > ros::Duration(lifetime))
		{
			for (int i = 0; i < points.size() - 1; i++)
			{
				points[i] = points[i+1];
			}
			points.pop_back();
			if (points.size() == 0)
			{
				break;
			}
		}
	}
	//ROS_INFO("%zu", points.size());
}


void MarkersPublisher::publishMarkers() // Publish vector of points as Marker Array
{
	visualization_msgs::MarkerArray marker_array;

	for (int i = 0; i < points.size(); i++)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = frame;
		marker.header.stamp = ros::Time::now();
		marker.ns = "basic_shapes";
    marker.id = i;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;


    marker.pose.position.x = points[i].x;
    marker.pose.position.y = points[i].y;
    marker.pose.position.z = points[i].z;

    // TODO: What is this check about?
		if ((points[i].v + volume_offset) < max_volume)
		{
			marker.scale.x = (points[i].v + volume_offset)/volume_divider;
    	marker.scale.y = (points[i].v + volume_offset)/volume_divider;
    	marker.scale.z = (points[i].v + volume_offset)/volume_divider;

  //    ROS_WARN("%f", marker.scale.x);
		}
		else
		{
			marker.scale.x = (max_volume + volume_offset)/volume_divider;
    	marker.scale.y = (max_volume + volume_offset)/volume_divider;
    	marker.scale.z = (max_volume + volume_offset)/volume_divider;

   //   ROS_ERROR("%f", marker.scale.x);
		}

		if (color_direction == "x") {
			marker.color = colorRainbow(((1.0f/(color_max-color_min))*(points[i].x)) - (color_min/(color_max-color_min)));
		} else if (color_direction == "y") {
			marker.color = colorRainbow(((1.0f/(color_max-color_min))*(points[i].y)) - (color_min/(color_max-color_min)));
		} else if (color_direction == "z") {
			marker.color = colorRainbow(((1.0f/(color_max-color_min))*(points[i].z)) - (color_min/(color_max-color_min)));
		} else {
			marker.color.r = 0.0f;
	    marker.color.g = 1.0f;
	    marker.color.b = 0.0f;
	    marker.color.a = 1.0;
		}

		marker.lifetime = ros::Duration(lifetime);

		marker_array.markers.push_back(marker);
	}

	markers_pub.publish(marker_array);
}


std_msgs::ColorRGBA MarkersPublisher::colorRainbow(float i) {
  int nZones = 5;       // divide sensing range into n equal zones
  float compressor = 2; // compresses the color ranges closer to origin
  int zone = nZones * i/(_sensingRange) * compressor; // ignore decimal value

  std_msgs::ColorRGBA color;
  switch (zone) {
    case 0:     // red zone
      color.r = 1.0f;
      color.g = color.b = 0.0f;
      break;
    case 1:     // yellow zone
      color.r = color.g = 1.0f;
      color.b = 0.0f;
      break;
    case 2:     // green zone
      color.g = 1.0f;
      color.r = color.b = 0.0f;
      break;
    case 3:     // cyan zone
      color.g = color.b = 1.0f;
      color.r = 0.0f;
      break;
    default:     // blue zone
      color.b = 1.0f;
      color.g = color.r = 0.0f;
      break;
   }
  color.a = 1.0f;
  return color;
}

} // namespace toposens_markers

