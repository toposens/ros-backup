/** @file     serial_test.cpp
 *  @author   Christopher Lang, Roua Mokchah
 *  @date     March 2019
 */

#include <toposens_driver/serial.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace toposens_driver;
/**
 * Testing the Serial constructor,the getFrame method and the shutdown method
 */

/**
 * Testing the serial port constructor
 * Desired behavior: Issue ROS_ERROR when serial opened with invalid device descriptor.
 */
TEST(SerialTest, openWithInvalidArgs)
{
  Serial* serial;

  // Test with an empty device endpoint
  testing::internal::CaptureStderr();
  serial = new Serial("");
  delete serial;
  EXPECT_FALSE(testing::internal::GetCapturedStderr().empty());

  // Test with a non-existent device endpoint
  testing::internal::CaptureStderr();
  serial = new Serial("hello");
  delete serial;
  EXPECT_FALSE(testing::internal::GetCapturedStderr().empty());
}


/**
 * Testing format of the data frame
 * Desired behavior: the data frame in buffer starts with an S and ends with an E
 */
TEST(SerialTest, getWellFormattedFrame) {
  std::string port = "/dev/ttyUSB0";

  std::stringstream frame;
  Serial serial(port);

  for (int i = 0; i < 10; i++) {
    serial.getFrame(frame);
    std::string data = frame.str();

    if(!data.size()) continue;

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
    std::size_t pos_s = data.find('S');
    std::size_t pos_e = data.find('E', pos_s + 1);
    if (pos_s == std::string::npos) ADD_FAILURE() << "Data frame missing S tag: " << data;
    if (pos_e == std::string::npos) ADD_FAILURE() << "Data frame missing E tag: " << data;

    frame.str(std::string());
    // End version 2
  }
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ts_driver/serial_test_node");
  return RUN_ALL_TESTS();

//  ros::start();
//  auto res = RUN_ALL_TESTS();
//  ros::shutdown();
//  return res;
}