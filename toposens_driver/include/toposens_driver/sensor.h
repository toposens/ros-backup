#ifndef SENSOR_H
#define SENSOR_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <toposens_driver/serial.h>
#include <toposens_msgs/TsScan.h>
#include <toposens_driver/TsConfig.h>

namespace toposens_driver
{
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
    void _cfgCallback(TsConfig &cfg, uint32_t level);

    std::string _port;          // serial port of sensor
    TsConfig _cfg;              // dynamic reconfigure params

    std::unique_ptr<dynamic_reconfigure::Server<TsConfig>> _srv;

		ros::Publisher _pub;
		std::unique_ptr<Serial> _serial;
		std::stringstream _data;
};
} // namespace toposens_driver

#endif
