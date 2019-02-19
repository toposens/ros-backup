#include "toposens_driver/sensor.h"


namespace toposens_driver
{

Sensor::Sensor(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  // Set up serial connection to sensor
  private_nh.getParam("port", _port);
	_serial = std::make_unique<Serial>(private_nh, _port);
	if (!_serial->isAlive()) throw "WTF Exception";

  // Check if sensor is busy self calibrating (every power on)
  if (_serial->isCalibrating()) {
    ROS_INFO("Auto calibration in progress...");
    while(_serial->isCalibrating()) continue;
    ROS_INFO("Sensor calibration successful");
  } else ROS_INFO("Sensor already calibrated");

  // Set up dynamic reconfigure to change sensor settings
  _srv = std::make_unique<dynamic_reconfigure::Server<TsDriverConfig> >(private_nh);
  dynamic_reconfigure::Server<TsDriverConfig>::CallbackType f;
  f = boost::bind(&Sensor::_reconfig, this, _1, _2);
  _srv->setCallback(f);

  // Advertise topic for Pointcloud-Messages
  _pub = nh.advertise<toposens_msgs::TsScan>(kPointsTopic, kTopicQueueSize);
  ROS_INFO("Publishing toposens data to /%s", kPointsTopic.c_str());
}


void Sensor::_init(void) {
  bool success = true;
  char cmd[kCmdBufferSize];

  Sensor::_cmd(cmd, kCmdSigStrength, _cfg.sig_strength);
  if (!_serial->send(cmd)) success = false;
  Sensor::_cmd(cmd, kCmdFilterSize, _cfg.filter_size);
  if (!_serial->send(cmd)) success = false;
  Sensor::_cmd(cmd, kCmdNoiseThresh, _cfg.noise_thresh);
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
      Sensor::_cmd(cmd, kCmdSigStrength, _cfg.sig_strength);
      break;
    case 1:
      Sensor::_cmd(cmd, kCmdFilterSize, _cfg.filter_size);
      break;
    case 2:
      Sensor::_cmd(cmd, kCmdNoiseThresh, _cfg.noise_thresh);
      break;
    case 3:
      Sensor::_cmd(cmd, kCmdBoostShortR, bst.short_range);
      break;
    case 4:
      Sensor::_cmd(cmd, kCmdBoostMidR, bst.mid_range);
      break;
    case 5:
      Sensor::_cmd(cmd, kCmdBoostLongR, bst.long_range);
      break;
    default:  // if function reaches here, then level is betwen 7 and 11
      Sensor::_cmd(cmd, kCmdVoxelLimits, 0);
  }

  if (_serial->send(cmd)) ROS_INFO("Sensor setting updated");
  else ROS_WARN("Settings update failed");
}


void Sensor::_cmd(char* out, const char* param, int val) {
  memset(out, 0, kCmdBufferSize);
  std::string format = "%c%s%05d\r";
  if (strcmp(param, kCmdVoxelLimits) != 0) {
    std::sprintf(out, format.c_str(), kCmdPrefix, param, val);
    return;
  }
  // at this point we know it is a voxel update
  // getting values directly from TsDriverConfig instead of passing 6 params
  format = "%c%s%05d%05d%05d%05d%05d%05d\r";
  TsDriverConfig::DEFAULT::VOXEL vxl = _cfg.groups.voxel;
  std::sprintf(out, format.c_str(), kCmdPrefix, param,
                vxl.y_max * -10, vxl.y_min * -10,
                vxl.z_min * 10, vxl.z_max * 10,
                vxl.x_min * 10, vxl.x_max * 10
              );    // ros -> ts :: x,y,z -> -y,z,x
}



bool Sensor::poll(void)
{
	if (!_serial->isAlive()) throw "WTF Exception";
  _serial->getFrame(_data);

	toposens_msgs::TsScan scan;
	scan.header.stamp = ros::Time::now();

	Sensor::_parse(scan);
  if (!scan.points.size()) return false;
  
	_pub.publish(scan);
	return true;
}


// // Parse string to points
//TODO: Describe parsing algorithm here?
// sample string: S100016P0000X00300Y00044Z00456V00065E
// sample 2: S000016P0000X-0415Y00010Z00257V00061P0000X-0235Y00019Z00718V00055
// P0000X-0507Y00043Z00727V00075P0000X00142Y00360Z01555V00052E
// This should be in another package like laserscan or whatever
// S100016: Frame header data; first bit is noise flag
// (whether signal had noice or not)
// P0000: Point header data; not used right now
// X, Y, Z: xyz coordinates relative to transducer in mm
// V: amplitude strength; from 0 to 256

// With new parsing algorithm, reduced parsing time from 12us to 3us.
// Parsing complexity decreased from O(n) to O(logn)

// NOTE!! For this algorithm to work, string data must be exactly in current format.
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
		std::string x_val = strData.substr(index + 1, val_length);
		
		index = strData.find('Y', index);
		std::string y_val = strData.substr(index + 1, val_length);		
		index = strData.find('Z', index);
		std::string z_val = strData.substr(index + 1, val_length);	
		index = strData.find('V', index);
		std::string v_val = strData.substr(index + 1, val_length);
		pt.intensity = std::stof(v_val)/100; // reduces points to normal size


	// Transforming from TS coordinate frame to ROS coordinate frame
	// TS frame: up is y+; right is x+; forward is z+
	// ROS frame: up is z+; right is y-; forward is x+
	// Therefore: ros_x = ts_z, ros_y = -ts_x; ros_z = ts_x
		if (pt.intensity > 0) {
			pt.location.x = std::stof(z_val)/1000;	// z becomes x
			pt.location.y = -std::stof(x_val)/1000;	// -x becomes y
			pt.location.z = std::stof(y_val)/1000;	// y becomes z
			scan.points.push_back(pt);
			pt = {};
		}
	}
	_data.str(std::string());
}


void Sensor::shutdown() 
{
	_serial.reset();
}

} // namespace toposens_driver
