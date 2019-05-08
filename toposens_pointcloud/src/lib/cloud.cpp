#include "toposens_pointcloud/cloud.h"

#include <ros/package.h>
#include <toposens_driver/sensor.h>



namespace toposens_pointcloud
{
/** Each incoming scan is converted to a new PointCloud message of
 *  template XYZI, which corresponds well with a TsPoint structure.
 *  A second persistent PointCloud structure holds the cumulative
 *  of all scans received during a run. At the end of each run, this
 *  cumulative data is saved as a PCD file in the package folder for
 *  subsequent use with other PCL tools (like pcl_octree_viewer).
 *  
 *  Prior to pointcloud conversion, each scan can also be optionally
 *  transformed to a different reference frame (usually, 'world' or
 *  'base_link' frame). This can be used, for instance, to efficiently
 *  map an area using sensor or odometry data.
 */
Cloud::Cloud(ros::NodeHandle nh, ros::NodeHandle private_nh)
{

  private_nh.param<std::string>("target_frame", target_frame, "toposens");

	_scans_sub = nh.subscribe(toposens_driver::kScansTopic, 100, &Cloud::_convert, this);
  _cloud_pub = nh.advertise<TsCloud>(kPointCloudTopic, 100);

  store = boost::make_shared<TsCloud>();
  pcl_conversions::toPCL(ros::Time::now(), store->header.stamp);
  store->header.frame_id = target_frame;
  store->height = 1;
}

/** Filename should be provided without extension. File is saved
 *  as a .pcd (hard-coded).
 */
void Cloud::save(std::string filename)
{
  if (!store->width) {
    ROS_WARN("No pointcloud data to save.");
    return;
  }
  pcl_conversions::toPCL(ros::Time::now(), store->header.stamp);
  std::string path = ros::package::getPath("toposens_pointcloud") + "/" + filename + ".pcd";
  if (pcl::io::savePCDFile(path, *store) == 0) {
    ROS_INFO("Saved latest point cloud data (%s)", path.c_str());
  }
}

/** Waits for a target frame transform to become available and maps
 *  incoming TsPoints to this frame. Transformed points are added
 *  to two pointcloud structures simultaneously, one instantaneous
 *  for publishing and the other persistent for storing.
 */
void Cloud::_convert(const toposens_msgs::TsScan::ConstPtr& msg)
{
  TsCloud::Ptr tc(new TsCloud);
  pcl_conversions::toPCL(msg->header.stamp, tc->header.stamp);
  tc->header.frame_id = target_frame;
  tc->height = 1;

  try
  {
    //@todo does this need to be done everytime or can it be done in constructor?
    _tf.waitForTransform(target_frame, msg->header.frame_id, msg->header.stamp, ros::Duration(0.5));
    
    for (auto it = msg->points.begin(); it != msg->points.end(); ++it)
    {
      if (it->intensity <= 0) continue;
      toposens_msgs::TsPoint p = _transform(*it, msg->header);
      tc->points.push_back(p);
      store->points.push_back(p);
    }
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO_STREAM(ex.what());
  }

  tc->width = tc->points.size();
  store->width += tc->width;
  _cloud_pub.publish(tc);

  Cloud::_addSensorMesh();
}

/** Converts TsPoint to an intermediary PointStamped message compatible
 *  with the transform listener. Builds a PCL-defined PointXYZI message
 *  from the resulting coordinates.
 *  Transformation is skipped if #target_frame is same as the point's
 *  current frame, and a simple conversion to PointXYZI is returned.
 */
toposens_msgs::TsPoint Cloud::_transform(toposens_msgs::TsPoint pt, std_msgs::Header h)
{
  geometry_msgs::PointStamped ps;
  ps.point = pt.location;

  if (h.frame_id != target_frame)
  {
    try
    {
      ps.header.frame_id = h.frame_id;
      ps.header.stamp = ros::Time::now();
      _tf.transformPoint(target_frame, ps, ps);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("TsPoint transformation failed: %s", ex.what());
    }
  }

  pt.location = ps.point;
  return pt;
}

void Cloud::_addSensorMesh(void)
{
  //@todo this needs to be implemented

  //  _rviz->publishMesh(og, "package://toposens_markers/meshes/Body.stl",
  //                     rviz_visual_tools::colors::DARK_GREY, 0.001, kMeshNs);
  //  _rviz->publishMesh(og, "package://toposens_markers/meshes/Cover.stl",
  //                     rviz_visual_tools::colors::DARK_GREY, 0.001, kMeshNs);
}


} // namespace toposens_pointcloud
