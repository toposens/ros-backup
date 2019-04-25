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
  protected:
    std::string mock_port, driver_port;
    ros::NodeHandle* private_nh;

    void SetUp()
    {
      private_nh = new ros::NodeHandle("~");
      private_nh->getParam("mock_port", mock_port);
      private_nh->getParam("port", driver_port);
    }

    void TearDown()
    {
      delete private_nh;
    }

    void _connect(std::string testPort, bool validPort, std::string onFail)
    {
      testing::internal::CaptureStderr();
      Serial* conn = new Serial(testPort);
      delete conn;

      if (validPort == testing::internal::GetCapturedStderr().empty()) {
        std::cerr << " pass" << std::endl;
      } else {
        std::cerr << " fail" << std::endl;
        ADD_FAILURE() << onFail;
      }
    }
};


/**
 * Testing the serial port constructor
 * Desired behavior: Issue ROS_ERROR when serial opened with invalid device descriptor.
 */
TEST_F(SerialTest, openInvalidPort)
{
  std::string failMsg = "Connecting to fictitious port did not result in failure.";

  std::cerr << "[TEST] Attempting connection to null port...";
  _connect("", false, failMsg);

  std::cerr << "[TEST] Attempting connection to non-existent port...";
  _connect("tty69", false, failMsg);
}


/**
 * Testing the serial port constructor
 * Desired behavior: Nothing written to cerr when serial opened with valid port.
 */
TEST_F(SerialTest, openValidPort)
{
  std::string failMsg = "Connecting to valid port resulted in failure.";

  std::cerr << "[TEST] Attempting connection to mock sensor port...";
  _connect(mock_port, true, failMsg);

  std::cerr << "[TEST] Attempting connection to mock driver port...";
  _connect(driver_port, true, failMsg);
}


/**
 * Testing Serial::getFrame.
 * Desired behavior: the extracted single TS data frame should be the same as the known published data
 * and should include an E tag.
 * If it's not the case : Issue Error.
 */
TEST_F(SerialTest, getFrameWellFormatted)
{
  const int mock_sensor = open(mock_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  const char tx_data[] = "S000016P0000X-0415Y00010Z00257V00061ES0000";

  write(mock_sensor, tx_data, sizeof(tx_data));

  Serial* serial = new Serial(driver_port);
  std::stringstream ss;
  serial->getFrame(ss);
  std::string rx_data = ss.str();


  EXPECT_STREQ(rx_data.c_str(),tx_data)<< "Written Data is supposed to be the same as Read Data!";

    if (rx_data.find('E') == std::string::npos) {
      ADD_FAILURE() << "Parsed data frame missing E tag: " << rx_data;
    }

  ss.str(std::string());
  delete serial;
  close(mock_sensor);
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
  const int mock_sensor = open(mock_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  
  EXPECT_TRUE(serial->send(data)) << "Error writing data to serial stream: " << data;
  while(n_bytes < 1) n_bytes = read(mock_sensor, &buffer, sizeof(buffer));

  // Size of data is number of chars + null terminator 
  EXPECT_EQ(n_bytes + 1, (int)sizeof(data)) << "Incorrect number of bytes received";

  delete serial;
  close(mock_sensor);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_driver_test");
  return RUN_ALL_TESTS();
}