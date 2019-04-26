#include "toposens_markers/plot.h"
#include <toposens_driver/sensor.h>


namespace toposens_markers
{   
/** A dynamic reconfigure server is set up to change marker scale
 *  and lifetime during runtime.
 */
Plot::Plot(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
	private_nh.param<std::string>("frame_id", _frame_id, "toposens");		//Frame for tf
  _rviz.reset(new rviz_visual_tools::RvizVisualTools(_frame_id,"/" + kMarkersTopic));
  _rviz->enableBatchPublishing();

  // Set up dynamic reconfigure to change marker settings
  _srv = std::make_unique<Cfg>(private_nh);
  Cfg::CallbackType f = boost::bind(&Plot::_reconfig, this, _1, _2);
  _srv->setCallback(f);

	// Subscribe to topic with TS scans
	_scans_sub = nh.subscribe(toposens_driver::kScansTopic, 100, &Plot::_sanitize, this);
  _sensingRange = 0;
}

/** Current implementation defines 2 marker parameters: scale and
 *  lifetime. On each trigger, the latest config data structure is
 *  copied locally and the user-defined global scale is updated for
 *  rviz rendering. No separate updating is done for the lifetime
 *  parameter as it is always retrieved directly from the cfg object
 *  whenever its value is needed in the code.
 */
void Plot::_reconfig(TsMarkersConfig &cfg, uint32_t level)
{
  if ((int)level > 2) {
    ROS_INFO("Update skipped: Parameter not recognized");
    return;
  }
  _cfg = cfg;
  _rviz->setGlobalScale(_cfg.scale);
  if (level == -1) ROS_INFO("Marker settings initialized");
  else ROS_INFO("Marker settings updated");
}

/** All incoming scans are stored in a double-ended queue, which
 *  allows for easy iteration, sequential arrangement of data and
 *  efficient chronological cleaning in susequent runs. On each
 *  trigger, all scans that have expired their lifetime are deleted.
 *  Rviz plotting is thereafter refreshed for this sanitzied data.
 */
void Plot::_sanitize(const toposens_msgs::TsScan::ConstPtr& msg)
{
  _scans.push_back(*msg);

  do {
    ros::Time oldest = _scans.front().header.stamp;
    // break loop if oldest timestamp is within lifetime
    if (ros::Time::now() - oldest < ros::Duration(_cfg.lifetime)) break;
    _scans.pop_front();
  } while(!_scans.empty());

  Plot::_plot();
}

/** Rviz provides no straightforward way of deleting individual
 *  markers once they have been plotted. Refreshing of display is
 *  achieved by clearing all markers on each run and re-plotting
 *  all data contained in the most recent sanitized copy of incoming
 *  scans.
 *
 *  Sensing range is also updated to keep track of the furthest point
 *  detected by the sensor. This is used for color-coding the markers.
 *  
 *  RVT (Rviz Visual Tools) wrappers are used for efficient Rviz
 *  plotting. Since batch publishing is enabled, RVT collects all
 *  markers to be visualized in a given update and publishes them
 *  to Rviz in one go when trigger() is called.
 */
void Plot::_plot(void) {
  _rviz->deleteAllMarkers();

  for (auto it1 = _scans.begin(); it1 != _scans.end(); ++it1) {
    std::vector<toposens_msgs::TsPoint> points = it1->points;

    for (auto it2 = points.begin(); it2 != points.end(); ++it2) {
      toposens_msgs::TsPoint pt = *it2;
      if (pt.location.x > _sensingRange) _sensingRange = pt.location.x;

      Eigen::Vector3d location = _rviz->convertPoint(pt.location);
      std_msgs::ColorRGBA color = _rviz->getColorScale(pt.location.x/_sensingRange);
      geometry_msgs::Vector3 scale = _rviz->getScale(_baseScale, pt.intensity);

      if (scale.x > 0) _rviz->publishSphere(location, color, scale, kPointsNs);
    }
  }
  // have to add mesh everytime since rviz has no way of
  // deleting all markers by namespace alone
  Plot::_addSensorMesh();
  _rviz->trigger();
}

/** Only adds the sensor mesh to RVT publishing queue. The actual
 *  publishing is done in the parent function which calls trigger().
 *  
 *  @todo scale the mesh to mm instead of scaling down from meters.
 *  @todo this orientation should be included in the stl files
 *  instead of being done here on each op.
 */ 
void Plot::_addSensorMesh(void)
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
