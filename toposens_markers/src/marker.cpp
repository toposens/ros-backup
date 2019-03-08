#include "toposens_markers/marker.h"

#include <toposens_driver/sensor.h>


namespace toposens_markers
{

Marker::Marker(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  private_nh.param<std::string>("frame", _frame, "toposens");   //Frame for tf

  _rviz.reset(new rviz_visual_tools::RvizVisualTools(_frame,"/" + kMarkersTopic));
  _rviz->enableBatchPublishing();

  // Set up dynamic reconfigure to change sensor settings
  _srv = std::make_unique<dynamic_reconfigure::Server<TsMarkersConfig> >(private_nh);
  dynamic_reconfigure::Server<TsMarkersConfig>::CallbackType f;
  f = boost::bind(&Marker::_reconfig, this, _1, _2);
  _srv->setCallback(f);

<<<<<<< HEAD
  // Subscribe to topic with ts points
  _sub = nh.subscribe(toposens_driver::kScansTopic, 100, &Marker::_sanitize, this);
=======
	// Subscribe to topic with ts points
	_sub = nh.subscribe(toposens_driver::kScansTopic, 100, &Marker::_sanitize, this);
>>>>>>> 2d019cd... Create Command class with overloaded constructors
  _sensingRange = 0;
}


void Marker::_reconfig(TsMarkersConfig &cfg, uint32_t level)
{
  if ((int)level > 2) {
    ROS_INFO("Update skipped: Parameter not recognized");
  } else {
    _cfg = cfg;
    _rviz->setGlobalScale(_cfg.scale);
    if (level == -1) ROS_INFO("Marker settings initialized");
    else ROS_INFO("Marker settings updated");
  }
}



void Marker::_sanitize(const toposens_msgs::TsScan::ConstPtr& msg)
{
  _scans.push_back(*msg);

  do {
    ros::Time oldest = _scans.front().header.stamp;
    // break loop if oldest timestamp is within lifetime
    if (ros::Time::now() - oldest < ros::Duration(_cfg.lifetime)) break;
    _scans.pop_front();
  } while(!_scans.empty());

  Marker::_plot();
}


void Marker::_plot(void) {
  _rviz->deleteAllMarkers();

  for (auto it1 = _scans.begin(); it1 != _scans.end(); ++it1) {
    std::vector<toposens_msgs::TsPoint> points = it1->points;

    for (auto it2 = points.begin(); it2 != points.end(); ++it2) {
      toposens_msgs::TsPoint pt = *it2;
      if (pt.location.x > _sensingRange) _sensingRange = pt.location.x;

      Eigen::Vector3d location = _rviz->convertPoint(pt.location);
      std_msgs::ColorRGBA color = _rviz->getColorScale(pt.location.x/_sensingRange);
      geometry_msgs::Vector3 scale = _rviz->getScale(_baseScale, pt.intensity);

      _rviz->publishSphere(location, color, scale, kPointsNs);
    }
  }
  // have to add mesh everytime since rviz has no way of
  // deleting all markers by namespace alone
  Marker::_addSensorMesh();
  _rviz->trigger();
}


void Marker::_addSensorMesh(void)
{
  geometry_msgs::Pose og;
  _rviz->generateEmptyPose(og);
  og.orientation.x = og.orientation.z = 1/sqrt(2);
  og.orientation.w = 0;

  _rviz->publishMesh(og, "package://toposens_markers/meshes/Body.stl",
    rviz_visual_tools::colors::DARK_GREY, 0.001, kMeshNs);
  _rviz->publishMesh(og, "package://toposens_markers/meshes/Cover.stl",
    rviz_visual_tools::colors::DARK_GREY, 0.001, kMeshNs);
}

} // namespace toposens_markers

