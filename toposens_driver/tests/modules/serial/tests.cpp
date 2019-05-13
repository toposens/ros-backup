/** @file     serial_test.cpp
 *  @author   Christopher Lang, Roua Mokchah, Adi Singh
 *  @date     March 2019
 */

#include <fcntl.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <toposens_driver/serial.h>

/**
 * Testing the Serial constructor,the serial::getFrame method and the serial::send method.
 */
using namespace toposens_driver;

class SerialTest : public ::testing::Test
{
public:
  const std::string TAG = "\033[36m[DriverSerialTest]\033[00m - ";

protected:
  std::string mock_sensor, driver_port;
  ros::NodeHandle* private_nh;


  void SetUp()
  {
    private_nh = new ros::NodeHandle("~");
    private_nh->getParam("mock_sensor", mock_sensor);
    private_nh->getParam("port", driver_port);
  }


  void TearDown()
  {
    delete private_nh;
  }

};


/**
 * Testing the serial port constructor
 * Desired behavior: Issue ROS_ERROR when serial opened with invalid device descriptor.
 */
TEST_F(SerialTest, openInvalidPort)
{
  std::cerr << TAG << "<openInvalidPort>\n";

  std::cerr << TAG << "\tAttempting connection to null port...";
  EXPECT_THROW(Serial(""), std::runtime_error);
  std::cerr << "done\n";

  std::cerr << TAG << "\tAttempting connection to non-existent port...";
  EXPECT_THROW(Serial("tty42"), std::runtime_error);
  std::cerr << "done\n";

  std::cerr << TAG << "</openInvalidPort>\n";
}


/**
 * Testing the serial port constructor
 * Desired behavior: Nothing written to cerr when serial opened with valid port.
 */
TEST_F(SerialTest, openValidPort)
{
  std::cerr << TAG << "<openValidPort>\n";

  std::cerr << TAG << "\tAttempting connection to virtual sensor port...";
  EXPECT_NO_THROW(Serial(mock_sensor.c_str()));
  std::cerr << "done\n";

  std::cerr << TAG << "\tAttempting connection to virtual driver port...";
  EXPECT_NO_THROW(Serial(driver_port.c_str()));
  std::cerr << "done\n";

  std::cerr << TAG << "</openValidPort>\n";
}


/**
 * Testing Serial::getFrame.
 * Desired behavior: the extracted single TS data frame should be the same as the known published data
 * and should include an E tag.
 * If it's not the case : Issue Error.
 */
TEST_F(SerialTest, getValidFrame)
{
  std::cerr << TAG << "<getValidFrame>\n";

  const char tx_data[] = "S000016P0000X-0415Y00010Z00257V00061ES0000";
  const int conn_fd = open(mock_sensor.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  std::cerr << TAG << "\tTx data: " << tx_data << "\n";
  std::cerr << TAG << "\tManually transmitting over " << mock_sensor << "...";
  write(conn_fd, tx_data, sizeof(tx_data));
  std::cerr << "done\n";

  std::cerr << TAG << "\tSerial::getFrame over " << driver_port << "...";
  Serial serial(driver_port);
  std::stringstream ss;
  serial.getFrame(ss);
  std::cerr << "done\n";

  std::string rx_data = ss.str();
  std::cerr << TAG << "\tRx data: " << rx_data << "\n";

  EXPECT_STREQ(rx_data.c_str(),tx_data);
  close(conn_fd);

  std::cerr << TAG << "</getValidFrame>\n";
}

/**
 * Testing Serial::send.
 * Use 2 interconnected mock ports: one to send some bytes and the other to read the sent data.
 * Desired behavior: Expect the return value of Serial::send to be true as long as data is written
 * to serial stream without error.
 * Expect the correct number of bytes to be received.
 */
TEST_F(SerialTest, sendValidBytes)
{
  std::cerr << TAG << "<sendValidBytes>\n";

  char tx_data[] = "CnWave00005\r";
  const int conn_fd = open(mock_sensor.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  std::cerr << TAG << "\tTx data: " << tx_data << "\n";
  std::cerr << TAG << "\tSerial::send over " << driver_port << "...";
  Serial serial(driver_port);
  EXPECT_TRUE(serial.send(tx_data));
  std::cerr << "done\n";

  std::cerr << TAG << "\tManually receiving over " << mock_sensor << "...";
  int n_bytes = 0;
  char buffer[20];
  while(n_bytes < 1) n_bytes = read(conn_fd, &buffer, sizeof(buffer));
  std::cerr << "done\n";

  std::cerr << TAG << "\tBytes received: " << n_bytes << "\n";
  EXPECT_EQ(n_bytes + 1, (int)sizeof(tx_data));   // Size of data is number of chars + null terminator 
  close(conn_fd);

  std::cerr << TAG << "</sendValidBytes>\n";
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_driver_serial_test");
  return RUN_ALL_TESTS();
}