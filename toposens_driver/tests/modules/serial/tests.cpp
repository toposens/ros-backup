/** @file     serial_test.cpp
 *  @author   Christopher Lang, Roua Mokchah, Adi Singh
 *  @date     March 2019
 */

#include <toposens_driver/serial.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <fcntl.h>

/**
 * Testing the Serial constructor,the serial::getFrame method and the serial::send method.
 */
using namespace toposens_driver;

class SerialTest : public ::testing::Test
{

public:
  const std::string TAG = "[DRIVER_SERIAL_TEST] - ";

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
  std::cerr << TAG << "Attempting connection to null port...";
  ASSERT_THROW(Serial(""), std::runtime_error);
  std::cerr << "done" << std::endl;

  std::cerr << TAG << "Attempting connection to non-existent port...";
  ASSERT_THROW(Serial("tty69"), std::runtime_error);
  std::cerr << "done" << std::endl;
}


/**
 * Testing the serial port constructor
 * Desired behavior: Nothing written to cerr when serial opened with valid port.
 */
TEST_F(SerialTest, openValidPort)
{
  std::cerr << TAG << "Attempting connection to mock sensor port...";
  EXPECT_NO_THROW(Serial(mock_sensor.c_str()));
  std::cerr << "done" << std::endl;

  std::cerr << TAG << "Attempting connection to mock driver port...";
  EXPECT_NO_THROW(Serial(driver_port.c_str()));
  std::cerr << "done" << std::endl;
}


/**
 * Testing Serial::getFrame.
 * Desired behavior: the extracted single TS data frame should be the same as the known published data
 * and should include an E tag.
 * If it's not the case : Issue Error.
 */
TEST_F(SerialTest, getFrameWellFormatted)
{
  const int conn_fd = open(mock_sensor.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  // @todo change this to input data dump for this test to make any meaningful sense
  const char tx_data[] = "S000016P0000X-0415Y00010Z00257V00061ES0000";
  std::cerr << TAG << "Tx Data: " << tx_data << std::endl;
  write(conn_fd, tx_data, sizeof(tx_data));

  Serial* serial = new Serial(driver_port);
  std::stringstream ss;

  serial->getFrame(ss);
  std::string rx_data = ss.str();
  std::cerr << "[TEST] Rx Data: " << rx_data << std::endl;
  
  //@todo maybe just do a check here against -1?
  if (rx_data.find('E') == std::string::npos) {
    ADD_FAILURE() << "Parsed data frame missing E tag: " << rx_data;
  }

  EXPECT_STREQ(rx_data.c_str(),tx_data)<< "Tx-Rx data mismatch";

  delete serial;
  close(conn_fd);
}

/**
 * Testing Serial::send.
 * Use 2 interconnected mock ports: one to send some bytes and the other to read the sent data.
 * Desired behavior: Expect the return value of Serial::send to be true as long as data is written
 * to serial stream without error.
 * Expect the correct number of bytes to be received.
 */
TEST_F(SerialTest, sendBytes)
{
  int n_bytes = 0;
  char buffer[20];
  char data[] = "hello world";

  Serial* serial = new Serial(driver_port);
  const int conn_fd = open(mock_sensor.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  
  EXPECT_TRUE(serial->send(data)) << "Error writing data to serial stream: " << data;
  while(n_bytes < 1) n_bytes = read(conn_fd, &buffer, sizeof(buffer));

  // Size of data is number of chars + null terminator 
  EXPECT_EQ(n_bytes + 1, (int)sizeof(data)) << "Incorrect number of bytes received";

  delete serial;
  close(conn_fd);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_driver_test");
  return RUN_ALL_TESTS();
}