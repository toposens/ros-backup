#include "toposens_driver/sensor.h"

namespace toposens_driver
{
/** A dynamic reconfigure server is set up to change sensor
 *  settings during runtime.
 */
Sensor::Sensor(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  std::string port;
  private_nh.getParam("port", port);
  private_nh.getParam("frame", _frame);

  // Set up serial connection to sensor
  _serial = std::make_unique<Serial>(port);
  if (!_serial->isAlive()) throw "WTF Exception";

  // @todo change this according to new firmware
  // Check if sensor is busy self calibrating (every power on)
//  if (_serial->isCalibrating()) {
//    ROS_INFO("Auto calibration in progress...");
//    while(_serial->isCalibrating()) continue;
 //   ROS_INFO("Sensor calibration successful");
//  } else ROS_INFO("Sensor already calibrated");

  // Check if sensor is busy self calibrating (every power on)
  if (_serial->isCalibrating()) {
    ROS_INFO("Auto calibration in progress...");
    while(_serial->isCalibrating()) continue;
    ROS_INFO("Sensor calibration successful");
  } else ROS_INFO("Sensor already calibrated");

  // Set up dynamic reconfigure to change sensor settings
  _srv = std::make_unique<Cfg>(private_nh);
  Cfg::CallbackType f = boost::bind(&Sensor::_reconfig, this, _1, _2);
  _srv->setCallback(f);

  // Publishing topic for TsScans
  _pub = nh.advertise<toposens_msgs::TsScan>(kScansTopic, kTopicQueueSize);
  ROS_INFO("Publishing toposens data to /%s", kScansTopic.c_str());
}

/** Only parameters within the root group of cfg ParameterGenerator
 *  broadcast their default values on initialization, so this method
 *  only transmits them to the sensor.
 *
 *  Parameters in any sub-groups broadcast their value as 0 on startup.
 *  It is also possible that sub-group params do not broadcast their
 *  value on startup, so polling the cfg server returns 0.
 *  This is likely a bug with the ROS dynamic reconfigure library.
*/
void Sensor::_init(void) {
  bool success = true;
  char cmd[kCmdBufferSize];

  Sensor::_getCmd(cmd, kCmdSigStrength, _cfg.sig_strength);
  if (!_serial->send(cmd)) success = false;
  Sensor::_getCmd(cmd, kCmdFilterSize, _cfg.filter_size);
  if (!_serial->send(cmd)) success = false;
  Sensor::_getCmd(cmd, kCmdNoiseThresh, _cfg.noise_thresh);
  if (!_serial->send(cmd)) success = false;

  if (success) ROS_INFO("Sensor settings initialized");
  else ROS_WARN("One or more settings failed to initialize");
}


void Sensor::_reconfig(TsDriverConfig &cfg, uint32_t level)
{
  if ((int)level > 11) {
    ROS_INFO("Update skipped: Parameter not recognized");
    return;
  }

  _cfg = cfg;
  if (level == -1) {
    Sensor::_init();
    return;
  }

//  ROS_INFO("Processing toposens reconfiguration");
  ROS_DEBUG("Updating parameter TSDriver %d", level);
  char cmd[kCmdBufferSize];
  TsDriverConfig::DEFAULT::BOOSTER bst = _cfg.groups.booster;

  switch(level) {
    case 0:
      Sensor::_getCmd(cmd, kCmdSigStrength, _cfg.sig_strength);
      break;
    case 1:
      Sensor::_getCmd(cmd, kCmdFilterSize, _cfg.filter_size);
      break;
    case 2:
      Sensor::_getCmd(cmd, kCmdNoiseThresh, _cfg.noise_thresh);
      break;
    case 3:
      Sensor::_getCmd(cmd, kCmdBoostShortR, bst.short_range);
      break;
    case 4:
      Sensor::_getCmd(cmd, kCmdBoostMidR, bst.mid_range);
      break;
    case 5:
      Sensor::_getCmd(cmd, kCmdBoostLongR, bst.long_range);
      break;
    default:  // if function reaches here, then level is betwen 7 and 11
      Sensor::_getCmd(cmd, kCmdVoxelLimits, 0);
  }

  if (_serial->send(cmd)) ROS_INFO("Sensor setting updated");
  else ROS_WARN("Settings update failed");
}

/** Necessary step to ensure commands are accepted by the firmware and
 *  settings are successfully updated.
 *
 *  Firmware defines commands in two formats, singular and dimensional.
 *  Singular commands expect one parameter value, dimensional commands
 *  expect XYZ limits defining a spatial cuboid. Currently, only the
 *  voxel filtering command is dimensionally formatted.
 *
 *  Note that all desired values are transmitted as zero-padded 5-byte
 *  strings, and the first byte is always reserved for the arithmetic
 *  sign: 0 for positive values, - for negative values.
 *
 *  Dimensional parameter values should be transformed to the TS-frame
 *  before transmission. The transform from TS- to ROS-frame is
 *  given by T(rt) = [(0, 0, 1), (-1, 0, 0), (0, 1, 0)].
 *
 *  Singular format:
 *  @n - Starts with char 'C'
 *  @n - 5 bytes defining firmware parameter to update
 *  @n - 5 bytes with desired parameter value
 *  @n - Terminating char '\r'
 *
 *  Dimensional format:
 *  @n - Starts with char 'C'
 *  @n - 5 bytes defining firmware parameter to update
 *  @n - 5 bytes with cuboid's lower X-limit
 *  @n - 5 bytes with cuboid's upper X-limit
 *  @n - 5 bytes with cuboid's lower Y-limit
 *  @n - 5 bytes with cuboid's upper Y-limit
 *  @n - 5 bytes with cuboid's lower Z-limit
 *  @n - 5 bytes with cuboid's upper Z-limit
 *  @n - Terminating char '\r'
 */
void Sensor::_getCmd(char* result, const char* param, int val) {
  memset(result, 0, kCmdBufferSize);
  std::string format = "%c%s%05d\r";
  if (strcmp(param, kCmdVoxelLimits) != 0) {
    std::sprintf(result, format.c_str(), kCmdPrefix, param, val);
    return;
  }
  // At this point we know it is a voxel update; getting values
  // directly from TsDriverConfig instead of passing six separate params
  format = "%c%s%05d%05d%05d%05d%05d%05d\r";
  TsDriverConfig::DEFAULT::VOXEL vxl = _cfg.groups.voxel;
  std::sprintf(result, format.c_str(), kCmdPrefix, param,
                vxl.y_max * -10, vxl.y_min * -10,
                vxl.z_min * 10, vxl.z_max * 10,
                vxl.x_min * 10, vxl.x_max * 10
              );  // ros --> ts :: x,y,z --> -y,z,x
}


// Reads into a private member datastream to avoid creating a buffer object on each poll
bool Sensor::poll(void)
{
  if (!_serial->isAlive()) throw "WTF Exception";
  _serial->getFrame(_data);

  // TODO: check for calibration bit here

  toposens_msgs::TsScan scan;
  scan.header.stamp = ros::Time::now();
  scan.header.frame_id = _frame;

  Sensor::_parse(scan);
  if (!scan.points.size()) return false;
  
  _pub.publish(scan);
  return true;
}


/** This O(log n) algorithm only works when the input data frame is
 *  exactly in the expected format. Char-by-char error checks are not
 *  implemented so as to increase parsing throughput.
 *
 *  A data frame corresponds to a single scan and has the following format:
 *  @n - Starts with char 'S'
 *  @n - 6 bytes of frame header info
 *  @n - Char 'P', indicates a measured point
 *  @n - 5 bytes of point header info
 *  @n - Char 'X', indicates x-coordinate of point
 *  @n - 5 bytes with measurement of x-coordinate
 *  @n - Char 'Y', indicates y-coordinate of point
 *  @n - 5 bytes with measurement of y-coordinate
 *  @n - Char 'Z', indicates z-coordinate of point
 *  @n - 5 bytes with measurement of z-coordinate
 *  @n - Char 'V', indicates intensity of signal
 *  @n - 5 bytes with measurement of signal intensity
 *  @n - ... Additional points in this scan ...
 *  @n - Ends with char 'E'
 *
 *  In the 6-byte long frame header:
 *  @n - First byte is set to 1 when measurements are taken in a noisy
 *  ambient environment (hence, likely inaccurate).
 *  @n - Second byte is currently unused.
 *  @n - Third byte is set to 1 while sensor calibration is in progress.
 *  @n - Fourth, fifth and sixth bytes show device address for I2C mode.
 *
 *  The 5-byte long pointer header is currently not used.
 *  The x-, y-, z-coordinates are calculated in mm relative to the sensor transducer.
 *  Signal intensity of the point is mapped to a scale of 0 to 255.
 *
 *  Sample frame: S000016P0000X-0415Y00010Z00257V00061P0000X-0235Y00019
 *  Z00718V00055P0000X-0507Y00043Z00727V00075P0000X00142Y00360Z01555V00052E
 *  @n Four points extracted: P1(-415, 10, 257, 61); P2(-235, 19, 718, 55);
 *  P3(-507, 43, 727, 75); P4(142, 360, 1555, 52)
 */
void Sensor::_parse(toposens_msgs::TsScan &scan)
{
  const int val_length = 5;
  std::size_t index = 0;
  toposens_msgs::TsPoint pt;

  //std::string strData = "S000016P0000X-0415Y00010Z00257V00061P0000X-0235Y00019Z00718V00055P0000X-0507Y00043Z00727V00075P0000X00142Y00360Z01555V00052E";
  std::string strData = _data.str();

  //  ROS_INFO_STREAM(strData);

  while (1) {
    index = strData.find('X', index);
    if (index == std::string::npos) break;
    std::string x_val, y_val, z_val, v_val;

    try {
      x_val = strData.substr(index + 1, val_length);
    
      index = strData.find('Y', index);
      y_val = strData.substr(index + 1, val_length);    
      index = strData.find('Z', index);
      z_val = strData.substr(index + 1, val_length);  
      index = strData.find('V', index);
      v_val = strData.substr(index + 1, val_length);
      pt.intensity = std::stof(v_val)/100; // reduces points to normal size


    // Transforming from TS coordinate frame to ROS coordinate frame
    // TS frame: up is y+; right is x+; forward is z+
    // ROS frame: up is z+; right is y-; forward is x+
    // Therefore: ros_x = ts_z, ros_y = -ts_x; ros_z = ts_x
      if (pt.intensity > 0) {
        pt.location.x = std::stof(z_val)/1000;  // z becomes x
        pt.location.y = -std::stof(x_val)/1000; // -x becomes y
        pt.location.z = std::stof(y_val)/1000;  // y becomes z
        scan.points.push_back(pt);
        pt = {};
      }
    } catch (const std::exception& e) {
      ROS_INFO_STREAM(strData);
      ROS_ERROR_STREAM(x_val);
      ROS_ERROR_STREAM(y_val);
      ROS_ERROR_STREAM(z_val);
      ROS_ERROR_STREAM(v_val);
      exit(EXIT_FAILURE);
    }
  }
  _data.str(std::string());
}

/** Deletes underlying serial and config server objects
 *  managed by class pointers. */
void Sensor::shutdown()
{
  _serial.reset();
  _srv.reset();
}

} // namespace toposens_driver
