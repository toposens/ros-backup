#ifndef TS_DRIVER_H
#define TS_DRIVER_H

#include <ros/ros.h>

#include <toposens_driver/ts_settings.h>
#include <toposens_driver/ts_serial.h>
#include <toposens_msgs/TsScan.h>


namespace ts_driver
{
class TsDriver
{
	public:
		TsDriver(ros::NodeHandle nh, ros::NodeHandle private_nh);
		~TsDriver() {}
		bool poll(void);
		void shutdown(void);

	private:
    void initSettings(void);
		void parseRawData(toposens_msgs::TsScan &scan);
		bool settingsCallback(toposens_driver::ts_settings::Request &req,
							toposens_driver::ts_settings::Response &res);

		// configuration parameters
		TsSerial::Config cfg_;
		ros::Publisher pub_;
		std::unique_ptr<TsSerial> serial_;
		std::stringstream data_;
};
}	// namespace ts_driver

#endif
