/** @file     serial.cpp
 *  @author   Adi Singh, Sebastian Dengler
 *  @date     January 2019
 */

#include "toposens_driver/serial.h"

#include <fcntl.h>
#include <ros/console.h>
#include <string.h>

namespace toposens_driver
{
/** Various termios struct flags are optimized for a connection
 *  that works best with the TS firmware. Any intrinsic flow
 *  control or bit processing is disabled.
 *
 *  Connection is a non-blocking thread that returns when at
 *  least 1 byte is received or 0.1s has passed since the last
 *  read operation.
 */
Serial::Serial(std::string port)
{
  _fd = -1;
  _port = port;

	// Open serial port to access sensor stream
	_fd = open(_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

	if (!isAlive()) {
		ROS_ERROR("Error opening connection at %s: %s", _port.c_str(), strerror (errno));
		return;
	}
  ROS_DEBUG("Toposens serial established with fd %d\n", _fd);

	// Set options of serial data transfer
	struct termios tty;
  memset(&tty, 0, sizeof(tty));

  // Get current attributes of termios structure
  if(tcgetattr(_fd, &tty) != 0) {
    ROS_WARN("Error retrieving attributes at %s: %s", _port.c_str(), strerror (errno));
    return;
  }

  // set I/O baud rate
  cfsetispeed(&tty, kBaud);
  cfsetospeed(&tty, kBaud);

  // enable reading, ignore ctrl lines, 8 bits per byte
  tty.c_cflag |= CREAD | CLOCAL | CS8;
  // turn off parity, use one stop bit, disable HW flow control
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

  // disable canonical mode and echoes
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL);

  // disable software flow control
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  // disable break condition handling
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK);
  // disable special data pre-processing
  tty.c_iflag &= ~(ISTRIP | INLCR | IGNCR | ICRNL);

  // disable special data post-processing
  tty.c_oflag &= ~(OPOST | ONLCR);

  // wait for at least 1 byte to be received
  // length of a valid empty data frame
  tty.c_cc[VMIN] = 1;
  // wait for 0.1s till data received
  tty.c_cc[VTIME] = 1;


	// Set attributes of termios structure
	if(tcsetattr(_fd, TCSANOW, &tty) != 0) {
		ROS_WARN("Error configuring device at %s: %s", _port.c_str(), strerror (errno));
		return;
	}

	ROS_DEBUG("Serial setings updated:\n  BaudRate = %d \n  DataBits = 8 \n  Parity = disabled", kBaud);
	tcflush(_fd, TCIFLUSH);			// Discard old data in RX buffer
	ROS_INFO("Device at %s ready for communication", _port.c_str());
}

/** Flushes out bits from the serial pipe and closes its
 *  corresponding linux file desciptor.
 */
Serial::~Serial(void)
{
  ROS_INFO("Closing serial connection...");

//	if (tcflush(_fd, TCIOFLUSH) || close(_fd) == -1) {
  if (close(_fd) == -1) {
		ROS_ERROR("Error closing serial connection: %s", strerror(errno));
		return;
	}
	ROS_INFO("Serial connection killed");
}

/** Uses the fact that open(3) returns -1 if it errors
 *  out while opening a serial stream.
 *
 *  @todo Since fd is never changed after setup, is there another way to probe?
 *  Is this function even useful?
 */
bool Serial::isAlive() {
  return (_fd != -1);
}

/** Reads incoming bytes to the string stream pointer till
 *  the firmware-defined frame terminator 'E' is reached.
 *  Returns if no data has been received for 1 second.
 */
void Serial::getFrame(std::stringstream &data)
{
  char buffer[2000];
	int nBytes = 0;
  ros::Time latest = ros::Time::now();
  
  do {
    memset(&buffer, '\0', sizeof(buffer));
    nBytes = read(_fd, &buffer, sizeof(buffer));
    if (nBytes < 1) continue;

 //   std::cerr << buffer;
    data << buffer;
    latest = ros::Time::now();
    // should this break instead when buffer contains E (at any position)
    if (buffer[nBytes-1] == 'E') break;
  } while (ros::Time::now() - latest < ros::Duration(1));
/*	do {
		memset(&buffer, '\0', sizeof(buffer));
		nBytes = read(_fd, &buffer, sizeof(buffer));
//    ROS_WARN("%d", nBytes);
    if (nBytes < 1) continue;
    data << buffer;
	} while (buffer[nBytes-1] != 'E');
*/
}

/** Note that this returns true as long as data is written to
 *  the serial stream without error. A success handshake from
 *  the sensor confirming the settings update is not currently
 *  checked for.
 *
 *  @todo Use a bit in the header payload to confirm settings update
 */
bool Serial::send(char* bytes)
{
	if (!isAlive()) {
    ROS_ERROR("Connection at %s unavailable: %s", _port.c_str(), strerror (errno));
    return false;
  }

  int tx_length = write(_fd, bytes, strlen(bytes));

  if (tx_length == -1) {
    ROS_ERROR("Failed to transmit %s: %s", bytes, strerror(errno));
    return false;
  }
	ROS_DEBUG("Bytes transmitted: %s", bytes);
  return true;
}

} // namespace toposens_driver