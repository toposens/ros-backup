#ifndef MAPPING_H
#define MAPPING_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <toposens_msgs/TsScan.h>

#include <pcl/octree/octree_pointcloud.h>



namespace toposens_pointcloud
{

typedef pcl::PointCloud<pcl::PointXYZI> TsCloud;

// Simple map based on odom data alone
class Cloud
{
public:
  Cloud(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~Cloud() {}
  void save(std::string filename); // for writing Octree data

private:
	ros::Subscriber _scans_sub;
  ros::Publisher _cloud_pub;  
	tf::TransformListener _tf;

	std::string fixed_frame; // make this same as data frame for no transform
  bool aggregate;

  // can add to octree simply by setting tc2 as input file
  // no need to show that here
  TsCloud::Ptr tc2;

  void _convert(const toposens_msgs::TsScan::ConstPtr& msg);
  pcl::PointXYZI _transform(std_msgs::Header h, toposens_msgs::TsPoint p);

};
} // namespace toposens_pointcloud

#endif
