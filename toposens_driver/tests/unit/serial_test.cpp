/** @file     serial_test.cpp
 *  @author   Christopher Lang, Roua Mokchah
 *  @date     March 2019
 */

#include <toposens_driver/serial.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <fcntl.h>

/**
 * Testing the Serial constructor,the getFrame method and the shutdown method
 */
using namespace toposens_driver;

class SerialTest : public ::testing::Test
{
  protected:
    std::string _sensor_port, _driver_port;
    ros::NodeHandle* private_nh;

    void SetUp()
    {
      private_nh = new ros::NodeHandle("~");
      private_nh->getParam("sensor_port", _sensor_port);
      private_nh->getParam("driver_port", _driver_port);
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
  _connect(_sensor_port, true, failMsg);

  std::cerr << "[TEST] Attempting connection to mock driver port...";
  _connect(_driver_port, true, failMsg);
}


/**
 * Testing format of the data frame
 * Desired behavior: the data frame in buffer starts with an S and ends with an E
 */
TEST_F(SerialTest, getFrameWellFormatted)
{
  const int _mock_sensor = open(_sensor_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  // @todo change this to input data dump for this test to make any meaningful sense
  const char tx_data[] = "S000016P0000X-0415Y00010Z00257V00061ES0000";
  std::cerr << "[TEST] Tx Data: " << tx_data << std::endl;
  write(_mock_sensor, tx_data, sizeof(tx_data));

  Serial* serial = new Serial(_driver_port);
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
  close(_mock_sensor);
}


int _print(std::string msg)
{
  std::cerr << "[INFO] " << msg << std::endl;
} 


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_driver_serial_test_node");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}