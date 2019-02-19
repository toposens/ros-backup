#include "toposens_driver/serial.h"

#include <fcntl.h>


namespace toposens_driver
{
Serial::Serial(ros::NodeHandle private_nh, std::string port)
{
	_fd = -1;
	kPortId = port.c_str();

	// Open serial port to access sensor stream
	_fd = open(kPortId, O_RDWR | O_NOCTTY | O_NDELAY);

	if (_fd == -1) {
		ROS_ERROR("Error opening connection at %s: %s", kPortId, strerror (errno));
		return;
	}
  ROS_DEBUG("Toposens serial established with fd %d\n", _fd);

	// Set options of serial data transfer
	struct termios tty;
  memset(&tty, 0, sizeof(tty));

  // Get current attributes of termios structure
  if(tcgetattr(_fd, &tty) != 0) {
    ROS_WARN("Error retrieving attributes at %s: %s", kPortId, strerror (errno));
    return;
  }
  
  // set I/O baud rate
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  // enable reading, ignore ctrl lines, 8 bits per byte
  tty.c_cflag |= CREAD | CLOCAL | CS8;  
  // turn off parity, use one stop bit, disable HW flow control
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

  // disable canonical mode, echoes and interrupts
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

  // disable software flow control
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  // disable break condition handling
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK);
  // disable special data pre-processing
  tty.c_iflag &= ~(ISTRIP | INLCR | IGNCR | ICRNL);

  // disable special data post-processing
  tty.c_oflag &= ~(OPOST | ONLCR);

  // return whenever any bytes are read
  tty.c_cc[VMIN] = 1;    // wait for at least 1 byte
  // wait upto 10 deciseconds (1s) for data
  tty.c_cc[VTIME] = 0;    //blocking thread


	// Set attributes of termios structure
	if(tcsetattr(_fd, TCSANOW, &tty) != 0) {
		ROS_WARN("Error configuring device at %s: %s", kPortId, strerror (errno));
		return;
	}

	ROS_DEBUG("Serial setings updated:\n  BaudRate = %d \n  DataBits = 8 \n  Parity = disabled", baud);
	tcflush(_fd, TCIFLUSH);			// Discard old data in RX buffer
	ROS_INFO("Device at %s ready for communication", kPortId);
}


Serial::~Serial(void)
{	
  ROS_INFO("Closing serial connection...");

	if (tcflush(_fd, TCIOFLUSH) | close(_fd) == -1) {
		ROS_ERROR("Error closing serial connection: %s", strerror(errno));
		return;
	}
	ROS_INFO("Serial connection killed");
}


bool Serial::isAlive() {
  return (_fd != -1);
}


bool Serial::isCalibrating() {
  char buffer = '\0';
  while (!buffer) read(_fd, &buffer, 1);
  return (buffer == '?');
}


void Serial::getFrame(std::stringstream &data) {
  char buffer[2000];
	int nBytes = 0;
	do     // Read until end of frame ('E')
	{
		memset(&buffer, '\0', sizeof(buffer));
		nBytes = read(_fd, &buffer, sizeof(buffer));
   // ROS_WARN("%d", nBytes);
    if (nBytes < 1) continue;
		data << buffer;
	} while (buffer[nBytes-1] != 'E');
}


bool Serial::send(char* data)
{
	if (_fd == -1) {
    ROS_ERROR("Connection at %s unavailable: %s", kPortId, strerror (errno));
    return false;
  }
	int tx_length = write(_fd, data, strlen(data) + 1);
	if (tx_length == -1) {
    ROS_ERROR("Failed to transmit %s: %s", data, strerror (errno));
    return false;
  }
	ROS_DEBUG("Bytes transmitted: %s", data);
  return true;
}

} // namespace toposens_driver