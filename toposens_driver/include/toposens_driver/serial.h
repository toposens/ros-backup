#ifndef SERIAL_H
#define SERIAL_H

#include <ros/ros.h>
#include <termios.h>

namespace toposens_driver
{
const unsigned int baud = 921600;

class Serial
{
	public:
		Serial(ros::NodeHandle private_nh, std::string port);
		~Serial();
    bool isAlive();
    bool isCalibrating();
		void getFrame(std::stringstream &data);
		bool send(char* data);

	private:
		int _fd;
		const char *kPortId;
};
} // namespace toposens_driver

#endif // SERIAL_H