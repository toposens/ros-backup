/** @file     sensor.h
 *  @author   Adi Singh, Sebastian Dengler
 *  @date     January 2019
 *  @brief    Converts raw sensor data to ROS friendly structures.
 *  Parses a TsScan from a single input data frame by extracting
 *  its header information and the vector of TsPoints contained
 *  in its payload. Messages are published to topic /ts_scans.
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <toposens_driver/serial.h>
#include <toposens_msgs/TsScan.h>
#include <toposens_driver/TsDriverConfig.h>

namespace toposens_driver
{
typedef dynamic_reconfigure::Server<TsDriverConfig> Cfg;
const std::string kScansTopic   = "ts_scans";
const int  kTopicQueueSize      = 100;

// @todo Separate this into a separate commands class
// The following commands are defined in TS firmware
const char kCmdPrefix           = 'C';
const int  kCmdBufferSize       = 100;
const char kCmdSigStrength[6]   = "nWave";
const char kCmdFilterSize[6]    = "filtr";
const char kCmdNoiseThresh[6]   = "dThre";
const char kCmdVoxelLimits[6]   = "goLim";
const char kCmdBoostShortR[6]   = "slop1";
const char kCmdBoostMidR[6]     = "slop2";
const char kCmdBoostLongR[6]    = "slop3";

/** Manages conversion of a single sensor data frame into a TsScan message.
 *  Also provides an interface for configuring sensor performance parameters.
 *
 *  A TsScan contains timestamped header information followed by a vector
 *  of TsPoints. A single TsPoint has a 3D location (x, y, z) and an
 *  associated intenstiy.
 */
class Sensor
{ 
  public:

    /** Initiates a serial connection and transmits default settings to sensor.
     *  @param nh Public nodehandle for pub-sub to ROS topics.
     *  @param private_nh Private nodehandle for retrieving launch parameters.
    */
    Sensor(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~Sensor() {}

    /** Retrieves raw sensor data frames and publishes TsScans extracted from them.
     *  @returns True if scan contains any valid data points. False for an empty scan.
     */
    bool poll(void);

    /** Shuts down serial connection to the sensor. */
    void shutdown(void);

  private:
    /** Transmits settings commands on startup with initial data
     *  from the config server.
     */
    void _init(void);

    /** Extracts TsPoints from the current data frame and reads them
     *  into the referenced TsScan object.
     *  @param scan Pointer to a TsScan for storing parsed data.
     */
    void _parse(toposens_msgs::TsScan &scan);

    /** Constructs a well-formed TS-defined settings command using
     *  the passed arguments.
     *  @param result Pointer to a char buffer where the result will be stored.
     *  @param param Setting name from the kCmd* list defined in this class.
     *  @param val Desired value of the setting. Ranges defined in .cfg file.
     */
    void _getCmd(char* result, const char* param, int val);

    /** Callback triggered when a parameter is altered on the dynamic
     *  reconfigure server.
     *  Determines which setting has changed and transmits the associated
     *  (well-formed) settings command to the serial stream.
     *  @param cfg Structure holding updated values of all parameters on server.
     *  @param level Indicates parameter that triggered the callback.
     */
    void _reconfig(TsDriverConfig &cfg, uint32_t level);

		/** Transforming from TS coordinate frame to ROS coordinate frame.
		 *  @param x_val Pointer to string containing x value in TS coordinate frame
		 *	@param y_val Pointer to string containing y value in TS coordinate frame
		 *	@param z_Val Pointer to string containing z value in TS coordinate frame
		 */
		void _transform(const std::string &x_val,
										const std::string &y_val,
										const std::string &z_val);

    std::string _frame;     /**< Frame ID assigned to TsScan messages.*/
    TsDriverConfig _cfg;    /**< Maintains current values of all config params.*/
    std::unique_ptr<Cfg> _srv;  /**< Pointer to config server*/

    ros::Publisher _pub;    /**< Topic for publishing TsScans.*/
    std::unique_ptr<Serial> _serial;  /**< Pointer for accessing serial functions.*/
    std::stringstream _data;  /**< Buffer for storing a raw data frame.*/

};
} // namespace toposens_driver

#endif
