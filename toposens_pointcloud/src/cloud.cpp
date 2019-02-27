#include "toposens_pointcloud/cloud.h"

#include <ros/package.h>
#include <toposens_driver/sensor.h>


namespace toposens_pointcloud
{

Cloud::Cloud(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  private_nh.param<std::string>("fixed_frame", fixed_frame, "odom");
	private_nh.param<bool>("aggregate", aggregate, false);

	_scans_sub = nh.subscribe(toposens_driver::kScanTopic, 100, &Cloud::_convert, this);
  _cloud_pub = nh.advertise<TsCloud>("ts_cloud", 100);

  tc2 = boost::make_shared<TsCloud>();
  tc2->header.frame_id = fixed_frame;
  tc2->height = 1;

}


void Cloud::_convert(const toposens_msgs::TsScan::ConstPtr& msg)
{
  //ROS_WARN_STREAM(*msg);

  TsCloud::Ptr tc(new TsCloud);
  tc->header.frame_id = fixed_frame;
  tc->height = 1;
  pcl_conversions::toPCL(msg->header.stamp, tc->header.stamp);

  std_msgs::Header header = msg->header;
  std::vector<toposens_msgs::TsPoint> points = msg->points;

  try {
    _tf.waitForTransform(fixed_frame, header.frame_id, header.stamp, ros::Duration(0.5));
    for (auto it2 = points.begin(); it2 != points.end(); ++it2) {
      pcl::PointXYZI p = _transform(header, *it2);
      tc->points.push_back(p);
      tc2->points.push_back(p);
    }
  } catch (tf::TransformException ex){
    ROS_INFO_STREAM(ex.what());
  }

  tc->width = tc->points.size();
  tc2->width += tc->width;
  _cloud_pub.publish (tc);

  ROS_WARN_STREAM(tc2->width);
  // TODO: save file on program termination instead
  if (tc2->width) save("toposens");   // without extension; will be saved as .pcd
}


pcl::PointXYZI Cloud::_transform(std_msgs::Header h, toposens_msgs::TsPoint p)
{
  geometry_msgs::PointStamped ps;
  ps.header = h;
  ps.point = p.location;
  _tf.transformPoint(fixed_frame, ps, ps);

  pcl::PointXYZI point(p.intensity);
  point.x = ps.point.x;
  point.y = ps.point.y;
  point.z = ps.point.z;
  return point;
}


void Cloud::save(std::string filename)
{
  char *path;
  sprintf(path, "%s/%s.pcd", ros::package::getPath("toposens_pointcloud"), filename);
  pcl::io::savePCDFile(path, *tc2);
}

} // namespace toposens_pointcloud
