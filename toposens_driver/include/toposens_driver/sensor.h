#ifndef SENSOR_H
#define SENSOR_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <toposens_driver/serial.h>
#include <toposens_msgs/TsScan.h>
#include <toposens_driver/TsDriverConfig.h>


namespace toposens_driver
{
const std::string kPointsTopic  = "ts_points";
const int  kTopicQueueSize      = 100;

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

class Sensor
{
	public:
		Sensor(ros::NodeHandle nh, ros::NodeHandle private_nh);
		~Sensor() {}
		bool poll(void);
		void shutdown(void);

	private:
    void _init(void);
		void _parse(toposens_msgs::TsScan &scan);
    void _cmd(char* out, const char* key, int val);
    void _reconfig(TsDriverConfig &cfg, uint32_t level);

    std::string _port;              // serial port of sensor
    TsDriverConfig _cfg;            // dynamic reconfigure params
    std::unique_ptr<dynamic_reconfigure::Server<TsDriverConfig>> _srv;

		ros::Publisher _pub;
		std::unique_ptr<Serial> _serial;
		std::stringstream _data;

};
} // namespace toposens_driver

#endif
