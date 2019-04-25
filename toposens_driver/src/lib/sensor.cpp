/** @file     sensor.cpp
 *  @author   Adi Singh, Sebastian Dengler
 *  @date     January 2019
 */

#include <toposens_driver/command.h>
#include "toposens_driver/sensor.h"
#define NOT_CALIBRATED 100.0


namespace toposens_driver
{
/** A dynamic reconfigure server is set up to configure sensor
 *  performance parameters during runtime.
 */
Sensor::Sensor(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  std::string port;
  private_nh.getParam("port", port);
  private_nh.getParam("frame_id", _frame_id);

  // Set up serial connection to sensor
  _serial = std::make_unique<Serial>(port);
  if (!_serial->isAlive()) throw "Error opening serial port!";

  _calibTempC = NOT_CALIBRATED;

  // Set up dynamic reconfigure to change sensor settings
  _srv = std::make_unique<Cfg>(private_nh);
  Cfg::CallbackType f = boost::bind(&Sensor::_reconfig, this, _1, _2);
  _srv->setCallback(f);

  // Publishing topic for TsScans
  _pub = nh.advertise<toposens_msgs::TsScan>(kScansTopic, kQueueSize);
  ROS_INFO("Publishing toposens data to /%s", kScansTopic);
}


/** Reads datastream into a private class variable to avoid creating
 *  a buffer object on each poll. Assumes serial connection is alive
 *  when function is called. The high frequency at which we poll
 *  necessitates that we dispense with edge-case checks.
 */
bool Sensor::poll(void)
{
  _scan.header.stamp = ros::Time::now();
  _scan.header.frame_id = _frame_id;
  _scan.points.clear();

  if (!_serial->isAlive()) throw "Serial connection has died!";
  _serial->getFrame(_buffer);
  Sensor::_parse(_buffer.str());

  if (_scan.points.empty()) return false;
  _pub.publish(_scan);
  
  _buffer.str(std::string());
  _buffer.clear();
  return true;
}

/** Deletes underlying serial and config server objects
 *  managed by class pointers.
 */
void Sensor::shutdown()
{
  _serial.reset();
  _srv.reset();
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

  Command cSig(Command::SigStrength, _cfg.sig_strength);
  if (!_serial->send(cSig.getBytes())) success = false;

  Command cFilter(Command::FilterSize, _cfg.filter_size);
  if (!_serial->send(cFilter.getBytes())) success = false;

  Command cNoise(Command::NoiseThresh, _cfg.noise_thresh);
  if (!_serial->send(cNoise.getBytes())) success = false;

  if (success) ROS_INFO("Sensor settings initialized");
  else ROS_WARN("One or more settings failed to initialize");
}

/** Determines which setting has changed and transmits the associated
 *  (well-formed) settings command to the serial stream. A unique level
 *  is assigned to each settings parameter in the cfg file. Current
 *  implementation defines 12 sensor performance parameters, indexed in
 *  cfg from 0 to 11. Config server triggers this method upon initialization
 *  with a special level of -1.
 */
void Sensor::_reconfig(TsDriverConfig &cfg, uint32_t level)
{
  if ((int)level > 11)
  {
    ROS_INFO("Update skipped: Parameter not recognized");
    return;
  }

  _cfg = cfg;
  if (level == -1) return Sensor::_init();

  Command* cmd;
  if     (level == 0) cmd = new Command(Command::SigStrength,  _cfg.sig_strength);
  else if(level == 1) cmd = new Command(Command::FilterSize,   _cfg.filter_size);
  else if(level == 2) cmd = new Command(Command::NoiseThresh,  _cfg.noise_thresh);
  else if(level == 3) cmd = new Command(Command::SNRBoostNear, _cfg.boost_near);
  else if(level == 4) cmd = new Command(Command::SNRBoostMid,  _cfg.boost_mid);
  else if(level == 5) cmd = new Command(Command::SNRBoostFar,  _cfg.boost_far);

  char* cmdString = cmd->getBytes();
  try
  {
    if (_serial->send(cmdString)) ROS_INFO("Sensor setting updated");
    else ROS_WARN("Settings update failed");
  } 
  catch (const std::exception& e)
  {
    ROS_ERROR("%s: %s", e.what(), cmdString);
  }
  delete cmd;
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
void Sensor::_parse(const std::string &frame)
{
  auto i = frame.begin();

  for (i; i < frame.end(); ++i) {
    // Find next X-tag in the frame
    while (*i != 'X') if (++i == frame.end()) return;

    try {
      toposens_msgs::TsPoint pt;
      pt.location.x = _toNum(++i) / 1000.0;

      if(*(++i) == 'Y') pt.location.y = _toNum(++i) / 1000.0;
      else throw std::invalid_argument("Expected Y-tag not found");

      if(*(++i) == 'Z') pt.location.z = _toNum(++i) / 1000.0;
      else throw std::invalid_argument("Expected Z-tag not found");

      if(*(++i) == 'V') pt.intensity = _toNum(++i) / 100.0;
      else throw std::invalid_argument("Expected V-tag not found");

      if (pt.intensity > 0) _scan.points.push_back(pt);

    } catch (const std::exception& e) {
      ROS_INFO("Skipped invalid point in stream");
      ROS_DEBUG("Error: %s in message %s", e.what(), frame.c_str());
    }
  }
}


/** Char is a valid number if its decimal range from ASCII value '0'
 *  falls between 0 and 9. Number is iteratively constructed through
 *  base-10 multiplication of valid digits and adding them together.
 *  The resulting number is cast to a float before returning.
 */
float Sensor::_toNum(auto &i)
{
  // Size of X, Y, Z, V data is always 5 bytes
  int abs = 0, factor = 1, length = 5;

  // If the first character is neither "-"
  // nor a number than throw an exception
  if (*i == '-') factor = -1;
  else if (*i != '0') throw std::invalid_argument("Invalid value char");

  while (--length) {
    int d = *(++i) - '0';
    if (d >= 0 && d <= 9) abs = abs*10 + d;
    else throw std::bad_cast();
  }
  return (float)(factor * abs);
}


/** Checks the calibration process indication bit in the data frame.
    S001020E => Calibration Empty Frame
    S000020E => Normal Empty Frame
  */
bool Sensor::_isCalibrating()
{
  _buffer.str(std::string());
  _buffer.clear();

  _serial->getFrame(_buffer);

  std::string data = _buffer.str().c_str();
  size_t frame_start = data.find('S');
  
  return (data[frame_start+3] == '1');
}

/** Performs sensor calibration for the given temperature.
  */
bool Sensor::calibrate(float ambientTempC)
{
  ROS_INFO("TS sensor calibrating for %3.1f C ...", ambientTempC);

  if(_calibTempC != ambientTempC){
    _calibTempC = NOT_CALIBRATED;
    Command cCalib(Command::CalibTemp, (int)(ambientTempC*10));
    _serial->send(cCalib.getBytes());
  }

  while(true){
    if(_isCalibrating()) _calibTempC = ambientTempC;
    else if(_calibTempC != NOT_CALIBRATED) break;
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("TS sensor calibration done.");
}

} // namespace toposens_driver
