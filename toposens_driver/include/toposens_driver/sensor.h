/** @file     sensor.h
 *  @author   Adi Singh, Sebastian Dengler
 *  @date     January 2019
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <toposens_driver/serial.h>
#include <toposens_msgs/TsScan.h>
#include <toposens_driver/TsDriverConfig.h>

/** @todo comment this? */
namespace toposens_driver
{
/** ROS topic for publishing TsScan messages. */
static const char kScansTopic[] = "ts_scans";
/** Maximum number of messages held in buffer for #kScansTopic. */
static const int kQueueSize = 100;

/** @brief  Converts raw sensor data to ROS friendly message structures.
 *  @details  Parses a TsScan from a single input data frame by extracting
 *  its header information and the vector of TsPoints contained in its payload.
 *  A TsScan contains timestamped header information followed by a vector
 *  of TsPoints. A single TsPoint has a 3D location (x, y, z) and an
 *  associated intenstiy. Messages are published to topic #kScansTopic.
 */
class Sensor
{
  public:
    /** Initiates a serial connection and transmits default settings to sensor.
     *  @param nh Public nodehandle for pub-sub ops on ROS topics.
     *  @param private_nh Private nodehandle for accessing launch parameters.
     */
    Sensor(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~Sensor() {}

    /** Retrieves raw sensor data frames and publishes TsScans extracted from them.
     *  @returns True if scan contains any valid data points. False for an empty scan.
     */
    bool poll(void);

    /** Shuts down serial connection to the sensor. */
    void shutdown(void);

    /** Performs sensor calibration for the given temperature.
     *  @param temperature value used for recalibration.
     *  @returns True if entering calibration mode.
     */
    bool calibrate(float ambientTempC);

  private:
    /** Structure generated from cfg file for storing local copy of sensor parameters.*/
    typedef dynamic_reconfigure::Server<TsDriverConfig> Cfg;

    /** Transmits settings commands on startup with initial data
     *  from the config server.
     */
    void _init(void);

    /** Callback triggered when a parameter is altered on the dynamic
     *  reconfigure server.
     *  Determines which setting has changed and transmits the associated
     *  (well-formed) settings command to the serial stream.
     *  @param cfg Structure holding updated values of all parameters on server.
     *  @param level Indicates parameter that triggered the callback.
     */
    void _reconfig(TsDriverConfig &cfg, uint32_t level);


    /** Extracts TsPoints from the current data frame and reads them
     *  into the referenced TsScan object.
     *  @param scan Pointer to a TsScan for storing parsed data.
     */
    void _parse(const std::string &frame);


  	/** Listens port whether sensor is in calibration mode.
  	 *	 @returns True if sensor is calibrating.
  	 */
  	bool _isCalibrating();
	
    /** Efficiently converts a char array representing a signed integer to
     *  its numerical value.
     *  @param i String iterator representing an integer value.
     *  @returns Signed float value after conversion.
     *  @throws std::bad_cast String contains non-numerical characters.
     */
    float _toNum(auto &i);

    std::string _frame_id;     /**< Frame ID assigned to TsScan messages.*/
    TsDriverConfig _cfg;    /**< Maintains current values of all config params.*/
    std::unique_ptr<Cfg> _srv;  /**< Pointer to config server*/

	  ros::Publisher _pub;    /**< Handler for publishing TsScans.*/
    std::unique_ptr<Serial> _serial;  /**< Pointer for accessing serial functions.*/
    std::stringstream _buffer;  /**< Buffer for storing a raw data frame.*/

    toposens_msgs::TsScan _scan;

    float _calibTempC;  /**< Latest calibrated temperature */

};

} // namespace toposens_driver

#endif
