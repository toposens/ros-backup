#ifndef TS_SERIAL_H
#define TS_SERIAL_H

#include <ros/ros.h>

namespace ts_driver 
{	
class TsSerial
{
	public:
		struct Config {
			std::string dev_id;				// serial port connected to sensor
	    std::string frame_id;           // tf frame ID
	    
	    // TODO: refactor these names and comment
	    std::string sig_length;
	    std::string filter_size;
	    std::string noise_threshold;
	    std::string goLim;
		};

		// set up connection in this constructor
		TsSerial(ros::NodeHandle private_nh, std::string dev_id);
		~TsSerial();

		bool getData(std::stringstream &dataStream);
    bool isAlive();

		bool writeData(std::string settings_cmd);
    bool updateSettings(Config cfg);

	private:
		int dev_fd_;
		const char *kDeviceId;
};
}   // namespace ts_driver

#endif // TS_SERIAL_H