#include "toposens_driver/serial.h"

#include <fcntl.h>
#include <termios.h>


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
	// TODO: comment all these flags
	struct termios kSerialSettings;
	tcgetattr(_fd, &kSerialSettings);
	kSerialSettings.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
	kSerialSettings.c_iflag = IGNPAR;
	kSerialSettings.c_oflag = 0;
	kSerialSettings.c_lflag = 0;

	// TODO: how about specifying a .c_cc[VMIN] and .c_cc[VTIME]
	// refer to https://www.xanthium.in/Serial-Port-Programming-on-Linux


	// Set attributes of termios structure
	if(tcsetattr(_fd, TCSANOW, &kSerialSettings) == -1) {
		ROS_ERROR("Error configuring device at %s: %s", kPortId, strerror (errno));
		return;
	}

	ROS_DEBUG("Serial setings updated:\n  BaudRate = 921600 \n  DataBits = 8 \n  Parity = ignore");
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


// make this more robust using velo function
bool Serial::get(std::stringstream &data) {
	// TODO: why this size?
  unsigned char BUF_RX[10000];
	int rx_length;
	ros::Time startTime	= ros::Time::now();

	do   // Read until end of frame ('E')
	{
		memset(BUF_RX, 0, 10000);
		rx_length = 0;
		rx_length = read(_fd, (void*)BUF_RX, 10000);
		data << BUF_RX;
    // maybe an operation like stringstream tellp()?
		// Return if no data received
		// Looks like a hack to find data recv.
		if (ros::Time::now() - ros::Duration(1) > startTime)
		{
			ROS_ERROR("No data received");
			return false;
		}
	} while (BUF_RX[rx_length-1] != 'E');
	return true;
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





/*
int InputSocket::getPacket(velodyne_msgs::VelodynePacket *pkt, const double time_offset)
{
  double time1 = ros::Time::now().toSec();

  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 1000; // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  while (true)
    {
      // Unfortunately, the Linux kernel recvfrom() implementation
      // uses a non-interruptible sleep() when waiting for data,
      // which would cause this method to hang if the device is not
      // providing data.  We poll() the device first to make sure
      // the recvfrom() will not block.
      //
      // Note, however, that there is a known Linux kernel bug:
      //
      //   Under Linux, select() may report a socket file descriptor
      //   as "ready for reading", while nevertheless a subsequent
      //   read blocks.  This could for example happen when data has
      //   arrived but upon examination has wrong checksum and is
      //   discarded.  There may be other circumstances in which a
      //   file descriptor is spuriously reported as ready.  Thus it
      //   may be safer to use O_NONBLOCK on sockets that should not
      //   block.

      // poll() until input available
      do
        {
          int retval = poll(fds, 1, POLL_TIMEOUT);
          if (retval < 0)             // poll() error?
            {
              if (errno != EINTR)
                ROS_ERROR("poll() error: %s", strerror(errno));
              return 1;
            }
          if (retval == 0)            // poll() timeout?
            {
              ROS_WARN("Velodyne poll() timeout");
              return 1;
            }
          if ((fds[0].revents & POLLERR)
              || (fds[0].revents & POLLHUP)
              || (fds[0].revents & POLLNVAL)) // device error?
            {
              ROS_ERROR("poll() reports Velodyne error");
              return 1;
            }
        } while ((fds[0].revents & POLLIN) == 0);

      // Receive packets that should now be available from the
      // socket using a blocking read.
      ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0],
                                packet_size,  0,
                                (sockaddr*) &sender_address,
                                &sender_address_len);

      if (nbytes < 0)
        {
          if (errno != EWOULDBLOCK)
            {
              perror("recvfail");
              ROS_INFO("recvfail");
              return 1;
            }
        }
      else if ((size_t) nbytes == packet_size)
        {
          // read successful,
          // if packet is not from the lidar scanner we selected by IP,
          // continue otherwise we are done
          if(devip_str_ != ""
             && sender_address.sin_addr.s_addr != devip_.s_addr)
            continue;
          else
            break; //done
        }

      ROS_DEBUG_STREAM("incomplete Velodyne packet read: "
                       << nbytes << " bytes");
    }

  // Average the times at which we begin and end reading.  Use that to
  // estimate when the scan occurred. Add the time offset.
  double time2 = ros::Time::now().toSec();
  pkt->stamp = ros::Time((time2 + time1) / 2.0 + time_offset);

  return 0;
}
*/
