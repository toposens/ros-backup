#ifndef MAPPING_H
#define MAPPING_H


#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <toposens_msgs/TsScan.h>
#include "toposens_pointcloud/octree.h"

#include <pcl/octree/octree_pointcloud.h>



namespace toposens_pointcloud
{
//This is how Ptr is impleement in PCL, maybe do this for octree
//typedef boost::shared_ptr<PointCloud<PointT> > Ptr;
typedef pcl::PointCloud<pcl::PointXYZI> TsCloud;

class GridMapping
{
public:
  GridMapping(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~GridMapping() {}

private:
	ros::Subscriber _sub;
  ros::Publisher _pub;

	tf::TransformListener tfListener;

	float octree_size;
	float octree_divisions;
	std::string fixed_frame;

	Octree *octree;
  pcl::octree::OctreePointCloud<pcl::PointXYZI> *ot;

  void _convert(const toposens_msgs::TsScan::ConstPtr& msg);
  pcl::PointXYZI _transform(std_msgs::Header h, toposens_msgs::TsPoint p);

};
} // namespace toposens_pointcloud

#endif
