/** @file     plot.h
 *  @author   Adi Singh, Sebastian Dengler
 *  @date     February 2019
 *  @brief    Visualizes published TsScans as native Rviz markers.
 *  Subscribes to a topic publishing TsScans and converts incoming
 *  data into Rviz Markers. Location, color and scale of a marker
 *  depend on payload of their corresponding TsPoint. The lifetime
 *  and scale of markers can be altered via dynamic reconfigure.
 */

#ifndef PLOT_H
#define PLOT_H

#include <deque>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <dynamic_reconfigure/server.h>

#include <toposens_driver/sensor.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <toposens_markers/TsMarkersConfig.h>
#include <toposens_msgs/TsScan.h>


namespace toposens_markers
{

static const std::string kMarkersTopic  = "ts_markers";
/**< Rviz namespace for plotted points.*/
static const std::string kPointsNs      = "TsPoints";
/**< Rviz namespace for sensor mesh.*/
static const std::string kMeshNs        = "TsSensor";
  

/** Handles lifetime management and realtime plotting of TsPoints on
*  Rviz.
*  Visual characteristics of a marker are defined as follows:
*  @n Location - 3D coordinates of TsPoint relative to origin, which
*  coincides with the sensor position.
*  @n Color - Relative z-distance from the sensor, green being the
*  closest points and red being the farthest.
*  @n Scale - Product of the base scale (hard-coded, same for all markers),
*  the user-defined global scale (dynamic, affects all markers equally)
*  and the intensity of an individual TsPoint.
*/
class Plot
{
  /** Universal scale affecting all markers equally.*/
  static const auto _baseScale = rviz_visual_tools::scales::LARGE;

  public:
    /** Subscribes to a TsScans topic and initializes visual tools for
     *  rviz plotting.
     *  @param nh Public nodehandle for pub-sub ops on ROS topics.
     *  @param private_nh Private nodehandle for accessing launch parameters.
     */
    Plot(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~Plot() {}

  private:
    /** Structure generated from cfg file for storing local copy of marker parameters.*/
    typedef dynamic_reconfigure::Server<TsMarkersConfig> Cfg;

    /** Callback triggered when a parameter is altered on the dynamic
     *  reconfigure server.
     *  @param cfg Structure holding updated values of parameters on server.
     *  @param level Indicates parameter that triggered the callback.
     */
    void _reconfig(TsMarkersConfig &cfg, uint32_t level);

    /** Keeps a local copy of incoming scans for plotting and clears
     *  out any stored data that is past its user-defined lifetime.
     *  @param msg Pointer to an incoming TsScan message.
     */
    void _sanitize(const toposens_msgs::TsScan::ConstPtr& msg);

    /** Generates rviz-friendly markers for all TsPoints stored locally
     *  at time of invocation.
     */
    void _plot(void);

    /** Adds a scaled TS sensor at rviz origin as a visual aid. */
    void _addSensorMesh(void);

    std::string _frame_id;          /**< Frame ID assigned to rviz Marker messages.*/
    TsMarkersConfig _cfg;        /**< Maintains current values of all config params.*/
    std::unique_ptr<Cfg> _srv;   /**< Pointer to config server*/

    ros::Subscriber _scans_sub;  /**< Handler for subscribing to TsScans.*/
    float _sensingRange;         /**< Largest z-distance measurement in subscribed data.*/
    std::deque<toposens_msgs::TsScan> _scans; /**< Data structure for storing incoming scans.*/
    rviz_visual_tools::RvizVisualToolsPtr _rviz;  /**< Helper for plotting markers.*/
};

} // namespace toposens_markers

#endif