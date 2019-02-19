#ifndef MARKER_H
#define MARKER_H

#include <deque>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <toposens_msgs/TsScan.h>
#include <toposens_markers/TsMarkersConfig.h>


namespace toposens_markers
{
const std::string kRvizTopic  = "ts_markers";
const auto _baseScale         = rviz_visual_tools::scales::LARGE;
const std::string kPointsNs   = "TsPoints";
const std::string kMeshNs     = "TsSensor";


class Marker
{
  public:
    Marker(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~Marker() {}

  private:
    void _reconfig(TsMarkersConfig &cfg, uint32_t level);
    void _sanitize(const toposens_msgs::TsScan::ConstPtr& msg);
    void _plot(void);
    void _addSensorMesh(void);

  	std::string _frame;
    float _sensingRange;
    TsMarkersConfig _cfg;            // dynamic reconfigure params
    std::unique_ptr<dynamic_reconfigure::Server<TsMarkersConfig>> _srv;

    ros::Subscriber _sub;
    // deque to support queue function + iteration
    std::deque<toposens_msgs::TsScan> _scans;
    rviz_visual_tools::RvizVisualToolsPtr _rviz;
};
} // namespace toposens_markers

#endif
