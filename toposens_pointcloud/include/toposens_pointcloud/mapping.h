/** @file     mapping.h
 *  @author   Adi Singh
 *  @date     March 2019
 */

#ifndef MAPPING_H
#define MAPPING_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

#include <toposens_msgs/TsPoint.h>
#include <toposens_msgs/TsScan.h>


POINT_CLOUD_REGISTER_POINT_STRUCT(
  toposens_msgs::TsPoint,
  (double, location.x, x)
  (double, location.y, y)
  (double, location.z, z)
  (float,  intensity,  i)
)

namespace toposens_pointcloud
{
  static const std::string kPointCloudTopic  = "ts_cloud";
  
  /** Templatized pointcloud containing all parsed TsPoints.*/
  typedef pcl::PointCloud<toposens_msgs::TsPoint> TsCloud;

/** @brief Demonstrates basic TF and PCL integrations for TsScan data.
 *  @details Subscribes to a topic publishing TsScans and converts 
 *  incoming data into PointCloud messages using PCL, optionally 
 *  mapped to another frame of reference. Visual behavior of the 
 *  pointcloud (scale, color, direction, lifetime, etc) can be 
 *  controlled via the PointCloud2 display in Rviz.
 */
class Mapping
{
  public:

    /** Subscribes to a TsScans topic and prepares a PointCloud structure
     *  for persistent storage.
     *  @param nh Public nodehandle for pub-sub ops on ROS topics.
     *  @param private_nh Private nodehandle for accessing launch parameters.
     */
    Mapping(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~Mapping() {} // @todo consider saving file on destructor

    /** Saves all pointcloud data contained in #store as a PCD file
     *  in root folder of the package.
     *  @param  filename  Desired name of PCD file (without extension). 
     */
    void save(std::string filename);

  private:
    /** Converts and broadcasts incoming scans as PointCloud messages.
     *  Maintains a local copy of all accumulated scans in their
     *  corresponding PointCloud form.
     *  @param  msg  Pointer to an incoming TsScan message.
     */
    void _convert(const toposens_msgs::TsScan::ConstPtr& msg);

    /** Maps a point to the reference frame defined in #target_frame.
     *  @param  p  TsPoint with coordinate values in original frame.
     *  @param  h  Header data of TsScan containing this point.
     *  @returns   PCL point with coordinate values in target frame. 
     */
    toposens_msgs::TsPoint _transform(toposens_msgs::TsPoint pt, std_msgs::Header h);

    /** Adds a scaled TS sensor at rviz origin as a visual aid. */
    void _addSensorMesh();

    std::string target_frame;     /**< Target frame for scan transformations.*/
    TsCloud::Ptr store;          /**< Collection of all pointclouds from a single run.*/

  	ros::Subscriber _scans_sub;  /**< Handler for subscribing to TsScans.*/
    ros::Publisher _cloud_pub;   /**< Handler for publishing PCL messages.*/
    ros::Publisher _mesh_pub;   /**< Handler for publishing sensor mesh.*/
  	tf::TransformListener _tf;   /**< Listener for frame mapping.*/
};

} // namespace toposens_pointcloud

#endif
