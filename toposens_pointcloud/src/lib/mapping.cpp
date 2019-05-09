#include "toposens_pointcloud/mapping.h"

#include <ros/package.h>
#include <toposens_driver/sensor.h>

 #include <visualization_msgs/MarkerArray.h>

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
Mapping::Mapping(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  private_nh.param<std::string>("target_frame", target_frame, "toposens");

	_scans_sub = nh.subscribe(toposens_driver::kScansTopic, 100, &Mapping::_convert, this);
  _cloud_pub = nh.advertise<TsCloud>(kPointCloudTopic, 100);
  _mesh_pub = nh.advertise<visualization_msgs::Marker>("ts_mesh", 100);
  Mapping::_addSensorMesh();

  store = boost::make_shared<TsCloud>();
  pcl_conversions::toPCL(ros::Time::now(), store->header.stamp);
  store->header.frame_id = target_frame;
  store->height = 1;
}

/** Filename should be provided without extension. File is saved
 *  as a .pcd (hard-coded).
 */
void Mapping::save(std::string filename)
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
void Mapping::_convert(const toposens_msgs::TsScan::ConstPtr& msg)
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

}

/** Converts TsPoint to an intermediary PointStamped message compatible
 *  with the transform listener. Builds a PCL-defined PointXYZI message
 *  from the resulting coordinates.
 *  Transformation is skipped if #target_frame is same as the point's
 *  current frame, and a simple conversion to PointXYZI is returned.
 */
toposens_msgs::TsPoint Mapping::_transform(toposens_msgs::TsPoint pt, std_msgs::Header h)
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


//@todo combine both STLs into one sensor mesh file
void Mapping::_addSensorMesh()
{
  auto end = ros::Time::now() + ros::Duration(1.0);
  while (ros::Time::now() < end) {
    if(_mesh_pub.getNumSubscribers() > 0) break;
    ros::Duration(0.1).sleep();
  }

  visualization_msgs::Marker body;
  body.header.frame_id = target_frame;
  body.header.stamp = ros::Time::now();
  body.ns = "TS3";
  body.id = 0;

  body.type = visualization_msgs::Marker::MESH_RESOURCE;
  // @todo this should come from toposens_description package
  body.mesh_resource = "package://toposens_pointcloud/meshes/Body.stl";
  body.scale.x = body.scale.y = body.scale.z = 0.001;
  body.color.r = body.color.b = body.color.g = 0.2;
  body.color.a = 1.0;

  _mesh_pub.publish(body);
}


} // namespace toposens_pointcloud
