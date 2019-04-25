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
 * Desired behavior: the extracted single TS data frame should include an S tag and an E tag.
 * If it's not the case : Issue Error.
 */
TEST_F(SerialTest, getFrameWellFormatted)
{
  const int mock_sensor = open(mock_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  // @todo change this to input data dump for this test to make any meaningful sense
  const char tx_data[] = "S000016P0000X-0415Y00010Z00257V00061ES0000";
  std::cerr << "[TEST] Tx Data: " << tx_data << std::endl;
  write(mock_sensor, tx_data, sizeof(tx_data));

  Serial* serial = new Serial(driver_port);
  std::stringstream ss;
  serial->getFrame(ss);
  std::string rx_data = ss.str();
  std::cerr << "[TEST] Rx Data: " << rx_data << std::endl;


  // @todo add more type of test data for get frame function
  // read data dump file line by line and pub/sub in loop


  // @todo can this be deleted now?
  /**
   * The following (commented) code tests for the desired behavior that the data frame in buffer starts with an S and
   * ends with an E. However, sometimes we receive incomplete frames, which we suspect to be a firmware problem.
   * Therefore we relaxed the test, such that the desired behavior is an S tag followed by an E tag in the buffer, which
   * ensures a successful application of the Sensor::_parse method
   */
  // Start Version 1
//      EXPECT_TRUE(data[0] == 'S')
//                  << "Data frame not prepended by S tag! " << data;
//
//      EXPECT_TRUE(data.back() == 'E')
//                  << "Data frame not closed by E tag!";
//      // End version 1

  // Start version 2
  // @todo is this not tested in sensor tests already?
  std::size_t index = rx_data.find('S');
  if (index == std::string::npos) {
    ADD_FAILURE() << "Parsed data frame missing S tag: " << rx_data;
  } else {
    index = rx_data.find('E', index + 1);
    if (index == std::string::npos) {
      ADD_FAILURE() << "Parsed data frame missing E tag: " << rx_data;
    }
  }
  // End version 2

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